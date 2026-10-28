#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
import sys
import os
import time
import serial
import struct
import subprocess

RESPONSE_PAYLOAD_LENGTHS = {
    0x11: {2},
    0x12: {4, 7},
    0x13: {19},
}

def modbus_crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def build_transfer_metadata(package):
    identity = bytearray(package[:16])
    if identity[13:16] == b"\x00\x00\x00":
        identity[13:16] = b"000"
    return (
        bytes(identity)
        + struct.pack(">I", len(package))
        + struct.pack(">H", modbus_crc16(package))
    )

def locate_firmware_payload(package):
    matches = []
    for size_order in ["little", "big"]:
        declared_size = int.from_bytes(package[16:20], size_order)
        if declared_size <= 0 or declared_size > len(package):
            continue
        offset = len(package) - declared_size
        if offset < 22:
            continue
        payload = package[offset:]
        calculated_crc = modbus_crc16(payload)
        for crc_order in ["little", "big"]:
            stored_crc = int.from_bytes(package[20:22], crc_order)
            if stored_crc == calculated_crc:
                matches.append(
                    (payload, offset, declared_size, calculated_crc, size_order, crc_order)
                )
    return matches[0] if matches else None

class BmsOta:
    def __init__(self, port, bin_path):
        self.port = port
        self.bin_path = bin_path
        self.ser = None
        self.bin_data = None
        self.firmware_data = None
        self.is_tty = sys.stdout.isatty()
        self.rx_buffer = bytearray()

    def log(self, msg, color="\033[0m"):
        if self.is_tty:
            print(f"{color}[BMS OTA] {msg}\033[0m", flush=True)
        else:
            print(f"[BMS OTA] {msg}", flush=True)

    def send_frame(self, cmd, data):
        length = len(data)
        frame = struct.pack(">BBB H", 0x01, 0x45, cmd, length) + data
        crc = modbus_crc16(frame)
        frame += struct.pack("<H", crc)
        self.ser.write(frame)
        self.ser.flush()
        return frame

    def enter_bootloader(self):
        frame = bytes.fromhex("01 06 A9 04 00 01 28 57")
        self.log("Step 1: Requesting APP to enter bootloader...")
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        self.ser.timeout = 1.0
        resp = self.ser.read(len(frame))
        if resp == frame:
            self.log("Bootloader jump command acknowledged")
        elif resp:
            self.log(
                f"Unexpected bootloader jump response: {resp.hex(' ')}",
                "\033[1;33m",
            )
        else:
            self.log(
                "No bootloader jump response; device may already be in bootloader",
                "\033[1;33m",
            )
        self.log("Waiting 10 seconds for bootloader startup...")
        time.sleep(10.0)
        self.ser.reset_input_buffer()
        self.rx_buffer.clear()

    def read_app_version(self, timeout=2.0):
        frame = bytes.fromhex("01 03 90 26 00 02 08 C0")
        self.ser.reset_input_buffer()
        self.rx_buffer.clear()
        self.ser.write(frame)
        self.ser.flush()

        deadline = time.monotonic() + timeout
        original_timeout = self.ser.timeout
        response = bytearray()
        try:
            while len(response) < 9:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.ser.timeout = remaining
                chunk = self.ser.read(9 - len(response))
                if not chunk:
                    return None
                response.extend(chunk)
        finally:
            self.ser.timeout = original_timeout

        if response[:3] != bytes.fromhex("01 03 04"):
            return None
        received_crc = struct.unpack("<H", response[-2:])[0]
        if received_crc != modbus_crc16(response[:-2]):
            return None
        return (
            int.from_bytes(response[3:5], "little"),
            int.from_bytes(response[5:7], "little"),
        )

    def read_app_version_string(self, timeout=2.0):
        """Vendor display version from 0xB007, e.g. 'v008 20260804-00'.

        This is the field that actually changes across APP upgrades; the
        0x9026 words can stay identical between APP versions (v007 and v008
        both report 0x0008 on LB-13S2P), so they cannot tell success from
        a no-op transfer."""
        body = bytes.fromhex("01 03 B0 07 00 08")
        frame = body + struct.pack("<H", modbus_crc16(body))
        self.ser.reset_input_buffer()
        self.rx_buffer.clear()
        self.ser.write(frame)
        self.ser.flush()

        deadline = time.monotonic() + timeout
        original_timeout = self.ser.timeout
        response = bytearray()
        try:
            while len(response) < 21:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.ser.timeout = remaining
                chunk = self.ser.read(21 - len(response))
                if not chunk:
                    return None
                response.extend(chunk)
        finally:
            self.ser.timeout = original_timeout

        if response[:3] != bytes.fromhex("01 03 10"):
            return None
        received_crc = struct.unpack("<H", response[-2:])[0]
        if received_crc != modbus_crc16(response[:-2]):
            return None
        return response[3:19].split(b"\x00")[0].decode("ascii", "replace") or None

    def verify_completion(self, version_before, app_before=None):
        self.log("Step 5: Verifying final status and APP recovery...")
        for elapsed in range(1, 9):
            time.sleep(1.0)
            self.log(f"Waiting for BMS reset: {elapsed}/8 seconds")

        for attempt in range(1, 11):
            version_after = self.read_app_version(timeout=1.0)
            if version_after:
                return self.report_app_version(version_before, version_after,
                                              app_before)
            self.log(f"APP recovery check ({attempt}/10)", "\033[1;33m")
            if attempt < 10:
                time.sleep(1.0)

        self.log("Transfer completed, but APP did not recover", "\033[1;31m")
        return False

    def report_app_version(self, version_before, version_after, app_before=None):
        sw_version, hw_version = version_after
        self.log(
            f"APP recovered: SW=0x{sw_version:04X}, HW=0x{hw_version:04X}",
            "\033[1;32m",
        )
        app_after = self.read_app_version_string(timeout=1.0)
        if app_before and app_after:
            self.log(f"APP display version: {app_before} -> {app_after}",
                     "\033[1;32m")
            if app_after == app_before:
                self.log("APP version did not change after transfer", "\033[1;31m")
                return False
            return True
        # Fallback for packs without 0xB007: compare the 0x9026 words (these
        # may not change across APP upgrades; the display string above is the
        # authoritative check when available).
        if version_before and version_after == version_before:
            self.log("APP version did not change after transfer", "\033[1;31m")
            return False
        return True

    def read_response(self, expected_cmd, timeout=1.0):
        deadline = time.monotonic() + timeout
        original_timeout = self.ser.timeout

        try:
            while time.monotonic() < deadline:
                frame_start = self.rx_buffer.find(b"\x01\x45")
                if frame_start < 0:
                    if self.rx_buffer[-1:] == b"\x01":
                        self.rx_buffer[:] = b"\x01"
                    else:
                        self.rx_buffer.clear()
                elif frame_start > 0:
                    del self.rx_buffer[:frame_start]

                if len(self.rx_buffer) >= 5:
                    cmd = self.rx_buffer[2]
                    length = int.from_bytes(self.rx_buffer[3:5], "big")
                    protocol_lengths = RESPONSE_PAYLOAD_LENGTHS.get(cmd)
                    if protocol_lengths is None or length not in protocol_lengths:
                        self.log(
                            f"Ignoring invalid response header: cmd=0x{cmd:02X}, "
                            f"length={length}",
                            "\033[1;33m",
                        )
                        del self.rx_buffer[0]
                        continue

                    frame_length = 7 + length
                    if len(self.rx_buffer) >= frame_length:
                        frame = bytes(self.rx_buffer[:frame_length])
                        received_crc = struct.unpack("<H", frame[-2:])[0]
                        expected_crc = modbus_crc16(frame[:-2])
                        if received_crc != expected_crc:
                            self.log(
                                f"Ignoring response with bad CRC: "
                                f"received=0x{received_crc:04X}, "
                                f"expected=0x{expected_crc:04X}",
                                "\033[1;33m",
                            )
                            del self.rx_buffer[0]
                            continue

                        del self.rx_buffer[:frame_length]
                        if cmd != expected_cmd:
                            self.log(
                                f"Ignoring late response for command 0x{cmd:02X}; "
                                f"waiting for 0x{expected_cmd:02X}",
                                "\033[1;33m",
                            )
                            continue
                        return frame[5:-2]

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.ser.timeout = remaining
                chunk = self.ser.read(1)
                if not chunk:
                    break
                self.rx_buffer.extend(chunk)
            return None
        finally:
            self.ser.timeout = original_timeout

    def run(self):
        if not os.path.isfile(self.bin_path):
            self.log(f"Error: firmware path is not a file: {self.bin_path}", "\033[1;31m")
            return False

        with open(self.bin_path, "rb") as f:
            self.bin_data = f.read()

        if len(self.bin_data) < 22:
            self.log("Error: firmware file is smaller than the 22-byte metadata", "\033[1;31m")
            return False

        payload_info = locate_firmware_payload(self.bin_data)
        if not payload_info:
            self.log(
                "Error: firmware payload length or CRC does not match the package header",
                "\033[1;31m",
            )
            return False
        (
            _firmware_payload,
            payload_offset,
            payload_size,
            payload_crc,
            size_order,
            crc_order,
        ) = payload_info
        self.firmware_data = self.bin_data

        self.log(f"Starting firmware upgrade: {self.bin_path} ({len(self.bin_data)} bytes)")
        self.log(
            f"Firmware payload: offset={payload_offset}, size={payload_size}, "
            f"CRC=0x{payload_crc:04X}, length_order={size_order}, crc_order={crc_order}"
        )
        transfer_crc = modbus_crc16(self.firmware_data)
        self.log(
            f"Transfer image: full package, size={len(self.firmware_data)}, "
            f"CRC=0x{transfer_crc:04X}"
        )

        try:
            serial_options = {"timeout": 1}
            if os.name == "posix":
                serial_options["exclusive"] = True
            self.ser = serial.Serial(self.port, 115200, **serial_options)

            version_before = self.read_app_version()
            if version_before:
                self.log(
                    f"Current APP version: SW=0x{version_before[0]:04X}, "
                    f"HW=0x{version_before[1]:04X}"
                )
            app_before = self.read_app_version_string()
            if app_before:
                self.log(f"Current APP display version: {app_before}")

            self.enter_bootloader()

            self.log("Step 2: Sending upgrade request...")
            req_data = b'\x30' * 6
            resp = None
            for attempt in range(1, 4):
                self.send_frame(0x11, req_data)
                resp = self.read_response(0x11)
                if resp:
                    break
                self.log(
                    f"Upgrade request timeout ({attempt}/3)",
                    "\033[1;33m",
                )

            if resp and (len(resp) < 2 or resp[0] != 0x00):
                status = resp[0] if resp else None
                error = resp[1] if resp and len(resp) > 1 else None
                error_names = {
                    0x01: "Low SOC",
                    0x02: "Update not supported",
                    0x03: "Abnormal protection state",
                    0x04: "Hardware mismatch",
                    0x05: "Software mismatch",
                }
                status_text = f"0x{status:02X}" if status is not None else "NULL"
                error_text = error_names.get(error, f"0x{error:02X}" if error is not None else "NULL")
                self.log(
                    f"Request rejected: status={status_text}, error={error_text}",
                    "\033[1;31m",
                )
                return False
            if not resp:
                self.log(
                    "No upgrade-request response; trying metadata for an already-active bootloader",
                    "\033[1;33m",
                )

            self.log("Step 3: Sending firmware metadata...")
            meta_data = build_transfer_metadata(self.firmware_data)
            metadata_ready = False
            for attempt in range(1, 6):
                self.send_frame(0x12, meta_data)
                resp = self.read_response(0x12)
                if not resp:
                    self.log(
                        f"Metadata response timeout ({attempt}/5)",
                        "\033[1;33m",
                    )
                    continue

                status = resp[0]
                if status == 0x00:
                    self.log("BMS reports that no update is needed", "\033[1;32m")
                    return True
                if status == 0x01:
                    self.log("BMS is erasing firmware, waiting...", "\033[1;33m")
                    time.sleep(1.0)
                    continue
                if status == 0x02:
                    self.log("BMS reports that the update is already complete", "\033[1;32m")
                    return True
                if status in [0x06, 0x08]:
                    metadata_ready = True
                    break

                status_names = {
                    0x04: "Data block receive check failure",
                    0x07: "Update failure",
                }
                detail = status_names.get(status, f"status 0x{status:02X}")
                self.log(f"Metadata verification failed: {detail}", "\033[1;31m")
                return False

            if not metadata_ready:
                self.log("Metadata verification timed out", "\033[1;31m")
                return False

            packet_size_code = resp[1]
            chunk_sizes = {1:64, 2:128, 3:240, 4:512, 5:1024}
            if packet_size_code not in chunk_sizes:
                self.log(
                    f"Unsupported packet size code: 0x{packet_size_code:02X}",
                    "\033[1;31m",
                )
                return False
            chunk_size = chunk_sizes[packet_size_code]
            requested_packet = int.from_bytes(resp[2:4], "big")
            if requested_packet != 0:
                self.log(
                    f"Bootloader requested packet {requested_packet}, but resume is not supported",
                    "\033[1;31m",
                )
                return False
            self.log(f"BMS requested chunk size: {chunk_size} bytes")
            self.log(
                f"Metadata accepted: status=0x{resp[0]:02X}, "
                f"requested={requested_packet}"
            )

            self.log("Step 4: Transferring data blocks...")
            total_size = len(self.firmware_data)
            total_blocks = (total_size + chunk_size - 1) // chunk_size
            if total_blocks > 0xFFFF:
                self.log("Firmware requires more than 65535 blocks", "\033[1;31m")
                return False
            self.log(f"Total firmware blocks: {total_blocks}")
            offset = 0
            pkt_idx = 0

            while offset < total_size:
                chunk = self.firmware_data[offset : offset + chunk_size]
                actual_len = len(chunk)
                header_info = self.bin_data[13:16]
                header_info += self.bin_data[12:13]
                header_info += struct.pack(">H", pkt_idx)
                header_info += struct.pack(">H", total_blocks)
                block_crc = modbus_crc16(chunk)
                header_info += struct.pack(">H", block_crc)

                if pkt_idx == 0 or pkt_idx == total_blocks - 1:
                    self.log(
                        f"Packet {pkt_idx} details: bytes={actual_len}, "
                        f"block_crc=0x{block_crc:04X}"
                    )

                packet_done = False
                for attempt in range(1, 4):
                    self.send_frame(0x13, header_info + chunk)
                    deadline = time.monotonic() + 1.0
                    detail = "NULL"
                    retry_delay = 0.0
                    while time.monotonic() < deadline:
                        remaining = deadline - time.monotonic()
                        resp = self.read_response(0x13, timeout=remaining)
                        if not resp:
                            break

                        status = resp[0]
                        requested_packet = int.from_bytes(resp[4:6], "big")
                        self.log(
                            f"Packet {pkt_idx} response: status=0x{status:02X}, "
                            f"requested={requested_packet}"
                        )

                        if resp[3] != self.bin_data[12] or resp[6:16] != self.bin_data[:10]:
                            detail = "response firmware identity does not match the target"
                            continue

                        if status == 0x00:
                            if requested_packet == pkt_idx + 1:
                                packet_done = True
                                break
                            if requested_packet <= pkt_idx:
                                detail = (
                                    f"waiting for packet {pkt_idx} write completion; "
                                    f"device requests {requested_packet}"
                                )
                                continue
                            detail = f"device unexpectedly requests packet {requested_packet}"
                            break

                        if status == 0x02:
                            if requested_packet == pkt_idx + 1:
                                packet_done = True
                                break
                            detail = (
                                f"packet {pkt_idx} was written, but device still "
                                f"requests {requested_packet}"
                            )
                            continue

                        status_names = {
                            0x01: "Check failed",
                            0x03: "Firmware blocking request in progress",
                            0x04: "Firmware size abnormal",
                            0x05: "Firmware information incomplete",
                            0x06: "Write failed",
                        }
                        detail = status_names.get(status, f"status 0x{status:02X}")
                        if status == 0x03:
                            retry_delay = 0.2
                        break

                    if packet_done:
                        break
                    self.log(
                        f"Packet {pkt_idx} not complete ({attempt}/3): {detail}",
                        "\033[1;33m",
                    )
                    if retry_delay > 0:
                        time.sleep(retry_delay)
                if not packet_done:
                    self.log(f"Packet {pkt_idx} failed after 3 attempts", "\033[1;31m")
                    return False

                offset += actual_len
                pkt_idx += 1
                progress = (offset / total_size) * 100
                if self.is_tty:
                    sys.stdout.write(
                        f"\r[BMS OTA] Progress: {pkt_idx}/{total_blocks} ({progress:.1f}%)"
                    )
                    sys.stdout.flush()
                else:
                    self.log(
                        f"Progress: {pkt_idx}/{total_blocks} ({progress:.1f}%)"
                    )

            if self.is_tty:
                sys.stdout.write("\n")
                sys.stdout.flush()
            self.log(
                f"All {total_blocks} firmware blocks acknowledged by the bootloader",
                "\033[1;32m",
            )
            return self.verify_completion(version_before, app_before)

        except (OSError, serial.SerialException) as exc:
            self.log(f"OTA I/O error: {exc}", "\033[1;31m")
            return False
        finally:
            if self.ser: self.ser.close()

if __name__ == "__main__":
    port_arg = sys.argv[1] if len(sys.argv) > 1 else "/etc/default/bms_daemon"
    if os.path.isfile(port_arg):
        with open(port_arg, "r") as f:
            for line in f:
                if "BMS_PORT=" in line:
                    port_arg = line.split("=")[1].strip()
                    break

    bin_file = sys.argv[2] if len(sys.argv) > 2 else "/opt/roboparty/lib/firmware/TWS_LB-13S2P_APP_v008_20260804-00.bin"
    ota = BmsOta(port_arg, bin_file)
    sys.exit(0 if ota.run() else 1)
