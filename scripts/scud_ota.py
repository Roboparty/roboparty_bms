#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0
# Copyright (C) 2026 wentywenty
"""SCUD (RP1) BMS 固件 OTA（RS485，115200 8N1）。

帧：FE FE | CMD | LEN | data(0..250) | CRC16H CRC16L | BB
帧校验 CRC16-CCITT（poly 0x1021、init 0），固件校验 CRC32（= CRC-32/ISO-HDLC）。
端序：0xC2 全部高字节在前；0xC3 应答两字段低字节在前。

命令：
    0xC0 [02 55|01 55]  进 Loader / 完成并跳 APP（文档的 01 00 无效）
    0xC1 []             查状态 → version(2) state(1) name(8/10/16)
    0xC2 [total_len(4) total_crc32(4)] → max_frame(1) page_size(2)
    0xC3 []             查进度 → received_len(4) received_crc32(4)
    0xC4 [data(N)]      传数据块 → ack
    0xC5 [0A 55]        分段提交 → ack

流程：0xC2 声明全镜像 → 0xC4 分块（末块短块直发）→ 每 2048 字节 0xC3+0xC5
→ 末次 0xC3 全量一致 → 01 55 紧跟提交跳 APP。详见 docs/scud-ota-flow.md。
"""

import os
import struct
import sys
import time

import serial

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

FRAME_HEAD = b"\xFE\xFE"
FRAME_TAIL = 0xBB
# Query requests (0xC1/0xC3) carry NO parameter: LEN=0 (required form).

CMD_CONTROL = 0xC0
CMD_STATUS = 0xC1
CMD_START = 0xC2
CMD_PROGRESS = 0xC3
CMD_BLOCK = 0xC4
CMD_COMMIT = 0xC5

STATUS_RESP_LEN = (11, 13, 19)  # version(2) state(1) name(8/10/16)：APP 回 11/13，Loader 回 19
START_RESP_LEN = 3              # max frame data len(1) page size(2)
PROGRESS_RESP_LEN = 8           # received len(4) received crc32(4)
ACK_RESP_LEN = 1

CTRL_JUMP_BOOT = b"\x02\x55"
CTRL_JUMP_APP = b"\x01\x55"         # finish-verify + jump APP（文档的 01 00 无效）
COMMIT_WORD = b"\x0A\x55"

# ACK 返回码 <-> 值，照抄 spec §4.1.1 表，不做翻译。
ACK_OK = 0x00
ACK_TRAIN_ERR = 0x01
ACK_WRITE_ERR = 0x02
ACK_ADDR_ERR = 0x03
ACK_CRC_ERR = 0x04
ACK_LEN_ERR = 0x05
ACK_TOTAL_LEN_ERR = 0x06
ACK_NO_CMD = 0x07
ACK_NAMES = {
    ACK_OK: "ACK_OK",
    ACK_TRAIN_ERR: "ACK_TRAIN_ERR",
    ACK_WRITE_ERR: "ACK_WRITE_ERR",
    ACK_ADDR_ERR: "ACK_ADDR_ERR",
    ACK_CRC_ERR: "ACK_CRC_ERR",
    ACK_LEN_ERR: "ACK_LEN_ERR",
    ACK_TOTAL_LEN_ERR: "ACK_TOTAL_LEN_ERR",
    ACK_NO_CMD: "ACK_NO_CMD",
}

STATE_IN_APP = 0x01
STATE_LOADER_IDLE = 0x02
STATE_UPGRADING = 0x03
STATE_UPGRADE_DONE = 0x04
STATE_NAMES = {
    0x01: "in APP",
    0x02: "in Loader, idle",
    0x03: "in Loader, upgrading",
    0x04: "in Loader, done",
}

# ---------------------------------------------------------------------------
# Transfer parameters
# ---------------------------------------------------------------------------

DEFAULT_CHUNK_SIZE = 128
MAX_CHUNK_SIZE = 250                # frame format hard limit (LEN < 250)
COMMIT_SEGMENT_BYTES = 2048         # 分段提交节奏（厂商工具实测；不依赖 page_size 字段）


# ---------------------------------------------------------------------------
# CRC helpers
# ---------------------------------------------------------------------------

def crc16_ccitt(data):
    """计算帧校验 CRC16-CCITT（poly 0x1021、init 0）。data: 待校验字节串。"""
    crc = 0
    for byte in data:
        crc = ((crc >> 8) | ((crc << 8) & 0xFFFF)) & 0xFFFF
        crc ^= byte
        crc ^= (crc & 0xFF) >> 4
        crc ^= (crc << 12) & 0xFFFF
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF
        crc &= 0xFFFF
    return crc


def crc32(data):
    """计算固件 CRC32（= CRC-32/ISO-HDLC）。data: 固件字节串。"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 & (0 - (crc & 1)))
    return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF


# ---------------------------------------------------------------------------
# OTA client
# ---------------------------------------------------------------------------

class ScudOta:
    def __init__(self, port, bin_path):
        self.port = port
        self.bin_path = bin_path
        self.ser = None
        self.firmware = None
        self.is_tty = sys.stdout.isatty()
        self._last_request = b""

    # ---- logging ----------------------------------------------------------

    def log(self, msg, color="\033[0m"):
        if self.is_tty:
            print(f"{color}[SCUD OTA] {msg}\033[0m", flush=True)
        else:
            print(f"[SCUD OTA] {msg}", flush=True)

    def log_progress(self, done, total):
        if self.is_tty:
            sys.stdout.write(
                f"\r[SCUD OTA] Progress: {done}/{total} "
                f"({(done / total) * 100:.1f}%)"
            )
            sys.stdout.flush()
        else:
            self.log(f"Progress: {done}/{total} ({(done / total) * 100:.1f}%)")

    # ---- frame layer ------------------------------------------------------

    def send_frame(self, cmd, payload):
        body = bytes([cmd, len(payload)]) + payload
        frame = FRAME_HEAD + body + struct.pack(">H", crc16_ccitt(body)) + bytes([FRAME_TAIL])
        self._last_request = frame
        self.ser.write(frame)
        self.ser.flush()
        return frame

    def read_response(self, cmd, expect_len, timeout=1.0):
        """逐字节收帧，返回目标帧载荷，超时返回 None。
        cmd: 目标命令字；expect_len: 期望载荷长度；timeout: 超时秒数。"""
        deadline = time.monotonic() + timeout
        lens = {expect_len} if isinstance(expect_len, int) else set(expect_len)
        stage = "head1"
        length = 0
        body = bytearray()
        crc = 0

        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            self.ser.timeout = left
            chunk = self.ser.read(1)
            if not chunk:
                continue  # 没字节就等到截止
            byte = chunk[0]

            match (stage, byte):
                case ("head1", 0xFE):
                    stage = "head2"
                case ("head2", 0xFE):
                    stage, body = "cmd", bytearray()
                case ("cmd", 0xFE):
                    pass  # 可能是下一帧头的第二字节，留在本态
                case ("cmd", b) if b == cmd:
                    stage = "len"
                case ("len", b) if b in lens:
                    length, body = b, bytearray()
                    stage = "payload" if b else "crc_hi"
                case ("payload", _):
                    body.append(byte)
                    if len(body) == length:
                        stage = "crc_hi"
                case ("crc_hi", _):
                    crc = byte << 8
                    stage = "crc_lo"
                case ("crc_lo", _):
                    crc |= byte
                    if crc == crc16_ccitt(bytes([cmd, length]) + bytes(body)):
                        stage = "tail"
                    else:
                        stage = "head2" if byte == 0xFE else "head1"
                case ("tail", 0xBB):
                    frame = FRAME_HEAD + bytes([cmd, length]) + bytes(body) + \
                        struct.pack(">H", crc) + bytes([FRAME_TAIL])
                    if frame != self._last_request:
                        return bytes(body)
                    stage = "head1"  # 自己的半双工回声
                # 垃圾/坏帧（含 LEN 不符）：就地丢弃，不吞其声称的载荷长度
                case (_, 0xFE):
                    stage = "head2"
                case _:
                    stage = "head1"

    # ---- commands ---------------------------------------------------------

    def control(self, action, timeout=1.5):
        """0xC0 控制命令，返回 ack 字节或 None。action: 控制字；timeout: 超时秒数。"""
        self.send_frame(CMD_CONTROL, action)
        resp = self.read_response(CMD_CONTROL, ACK_RESP_LEN, timeout)
        return resp[0] if resp else None

    def _query(self, cmd, resp_len, timeout=2.0):
        """查询请求（LEN=0 无参数），返回响应载荷或 None。
        cmd: 命令字；resp_len: 期望响应长度；timeout: 超时秒数。"""
        self.send_frame(cmd, b"")
        return self.read_response(cmd, resp_len, timeout)

    def get_status(self):
        """0xC1 查询状态，返回 (version, state, name) 或 None。"""
        resp = self._query(CMD_STATUS, STATUS_RESP_LEN)
        if resp is None:
            return None
        version = struct.unpack(">H", resp[0:2])[0]
        state = resp[2]
        name = resp[3:].split(b"\x00")[0].decode("ascii", "replace").strip()
        return version, state, name

    def get_progress(self):
        """0xC3 查询进度，返回 (received_len, received_crc32) 或 None。
        实测响应字段为小端。"""
        resp = self._query(CMD_PROGRESS, PROGRESS_RESP_LEN)
        if resp is None:
            return None
        return struct.unpack("<II", resp[0:8])

    def start_upgrade(self, total_len, total_crc32, timeout=15.0):
        """0xC2 开始升级（请求 LEN=8 [total_len total_crc32]），返回
        (max_frame_len, page_size) 或 None。
        total_len: 固件总长；total_crc32: 固件 CRC32；timeout: 超时秒数。"""
        payload = struct.pack(">II", total_len, total_crc32)
        for _ in range(2):
            self.send_frame(CMD_START, payload)
            resp = self.read_response(CMD_START, START_RESP_LEN, timeout)
            if resp is not None:
                max_frame = resp[0]
                page_size = struct.unpack(">H", resp[1:3])[0]
                return max_frame, page_size
        return None

    def send_block(self, data, timeout=2.0):
        """0xC4 发送数据块，返回 ack 字节或 None。data: 块数据；timeout: 超时秒数。"""
        self.send_frame(CMD_BLOCK, data)
        resp = self.read_response(CMD_BLOCK, ACK_RESP_LEN, timeout)
        return resp[0] if resp else None

    def commit(self, timeout=30.0):
        """0xC5 提交，返回 ack 字节或 None。timeout: 超时秒数。"""
        self.send_frame(CMD_COMMIT, COMMIT_WORD)
        resp = self.read_response(CMD_COMMIT, ACK_RESP_LEN, timeout)
        return resp[0] if resp else None

    # ---- helpers ----------------------------------------------------------

    @staticmethod
    def ack_name(code):
        return ACK_NAMES.get(code, f"0x{code:02X}") if code is not None else "no response"

    def wait_for_state(self, want, tries=15, delay=1.0):
        last = None
        for _ in range(tries):
            status = self.get_status()
            if status:
                last = status
                if status[1] == want:
                    return status
            time.sleep(delay)
        return last

    # ---- main flow --------------------------------------------------------

    def run(self):
        if not os.path.isfile(self.bin_path):
            self.log(f"Error: firmware path is not a file: {self.bin_path}", "\033[1;31m")
            return False
        with open(self.bin_path, "rb") as f:
            self.firmware = f.read()
        total = len(self.firmware)
        if total == 0:
            self.log("Error: firmware file is empty", "\033[1;31m")
            return False

        self.log(f"Starting firmware upgrade: {self.bin_path} ({total} bytes)")
        self.log(f"Local CRC32 (reference code): 0x{crc32(self.firmware):08X}")

        try:
            options = {"timeout": 1}
            if os.name == "posix":
                options["exclusive"] = True
            self.ser = serial.Serial(self.port, 115200, **options)

            # -- before: version / state / name -----------------------------
            before = self.get_status()
            if before is None:
                self.log("Error: no response to 0xC1 status query", "\033[1;31m")
                return False
            version_before, state, name = before
            self.log(
                f"Current: SW=0x{version_before:04X}, state=0x{state:02X} "
                f"({STATE_NAMES.get(state, '?')}), device='{name}'"
            )

            # -- enter Loader ----------------------------------------------
            if state == STATE_IN_APP:
                self.log("Step 1: Requesting jump to BOOT (0xC0 02 55)...")
                ack = self.control(CTRL_JUMP_BOOT, timeout=1.5)
                if ack is not None:
                    self.log(f"Jump request acknowledged ({self.ack_name(ack)})")
                else:
                    self.log(
                        "No jump response; device may already be rebooting",
                        "\033[1;33m",
                    )
                status = self.wait_for_state(STATE_LOADER_IDLE, tries=15, delay=1.0)
                if not status or status[1] not in (
                    STATE_LOADER_IDLE, STATE_UPGRADING, STATE_UPGRADE_DONE
                ):
                    self.log("Error: device did not reach the Loader", "\033[1;31m")
                    return False
                self.log(f"Loader ready (state=0x{status[1]:02X})")
            elif state in (STATE_UPGRADING, STATE_UPGRADE_DONE):
                self.log(
                    f"Device already in an upgrade state (0x{state:02X}); "
                    "restarting the transfer from scratch",
                    "\033[1;33m",
                )
            else:
                self.log("Step 1: Device already in Loader")

            return self.transfer_and_finish(version_before)

        except (OSError, serial.SerialException) as exc:
            self.log(f"OTA I/O error: {exc}", "\033[1;31m")
            return False
        finally:
            if self.ser:
                self.ser.close()

    def transfer_and_finish(self, version_before):
        total = len(self.firmware)

        # -- Step 2: start upgrade ------------------------------------------
        self.log("Step 2: Starting upgrade (0xC2)...")
        total_crc = crc32(self.firmware)
        started = self.start_upgrade(total, total_crc)
        if started is None:
            self.log("Error: no accepted 0xC2 request layout", "\033[1;31m")
            return False
        max_frame, page_size = started
        chunk_size = max_frame if 1 <= max_frame <= MAX_CHUNK_SIZE else DEFAULT_CHUNK_SIZE
        if chunk_size != max_frame:
            self.log(
                f"Device returned odd max frame length {max_frame}; "
                f"using {DEFAULT_CHUNK_SIZE}",
                "\033[1;33m",
            )
        self.log(f"Negotiated chunk size: {chunk_size} bytes, page_size field: {page_size}")

        if self.transfer_blocks(chunk_size) != "ok":
            return False

        # -- Step 4: full verify -------------------------------------------
        self.log("Step 4: Verifying...")
        progress = self.get_progress()
        if progress is None:
            self.log("Error: no response to 0xC3 progress query", "\033[1;31m")
            return False
        recv_len, recv_crc = progress
        self.log(f"Device reports: received={recv_len}, crc32=0x{recv_crc:08X}")
        total_crc = crc32(self.firmware)
        if recv_len != total or recv_crc != total_crc:
            self.log(
                f"Error: device-side check mismatch (expected len={total}, "
                f"crc32=0x{total_crc:08X})",
                "\033[1;31m",
            )
            return False
        self.log("Device-side length and CRC32 match", "\033[1;32m")

        # -- Step 5: finish + jump APP (0xC0 01 55，紧跟最终提交) -----------
        self.log("Step 5: Finish and jump to APP (0xC0 01 55)...")
        for attempt in range(1, 6):
            ack = self.control(CTRL_JUMP_APP, timeout=2.0)
            if ack == ACK_OK:
                self.log("Finish acknowledged", "\033[1;32m")
                break
            self.log(f"Finish attempt {attempt}: {self.ack_name(ack)}", "\033[1;33m")
            if ack == ACK_TOTAL_LEN_ERR:
                self.log("升级会话已清空（ACK_TOTAL_LEN_ERR）；需完整重跑一次", "\033[1;31m")
                return False
            time.sleep(0.5)
        else:
            self.log("Error: no ack to 0xC0 01 55", "\033[1;31m")
            return False

        status = self.wait_for_state(STATE_IN_APP, tries=15, delay=1.0)
        if not status or status[1] != STATE_IN_APP:
            self.log("Transfer completed, but APP did not recover", "\033[1;31m")
            return False
        version_after = status[0]
        self.log(
            f"APP recovered: SW=0x{version_after:04X} (was 0x{version_before:04X})",
            "\033[1;32m",
        )
        if version_after == version_before:
            self.log("APP version unchanged（同版本重刷，预期）", "\033[1;33m")
        return True

    def transfer_blocks(self, chunk_size):
        """0xC4 循环，每 COMMIT_SEGMENT_BYTES 字节做 0xC3+0xC5 分段提交，
        返回 "ok" | "fail"。chunk_size: 每块字节数。"""
        total = len(self.firmware)
        total_blocks = (total + chunk_size - 1) // chunk_size
        self.log("Step 3: Transferring firmware blocks...")
        self.log(f"Total firmware blocks: {total_blocks}")
        self.log(
            "Point of no return: firmware data now flows to the device; "
            "an interrupted run must be redone from the start",
            "\033[1;33m",
        )

        offset = 0
        next_seg = COMMIT_SEGMENT_BYTES
        for index in range(total_blocks):
            block = self.firmware[offset : offset + chunk_size]
            if not self.send_block_checked(index, offset, block, chunk_size):
                return "fail"
            offset += len(block)
            self.log_progress(index + 1, total_blocks)
            if offset >= next_seg or offset == total:
                if not self.segment_commit(offset):
                    return "fail"
                next_seg += COMMIT_SEGMENT_BYTES

        if self.is_tty:
            sys.stdout.write("\n")
            sys.stdout.flush()
        self.log(f"All {total_blocks} firmware blocks acknowledged", "\033[1;32m")
        return "ok"

    def segment_commit(self, sent):
        """分段 0xC3 校验 + 0xC5 提交，返回 True/False。sent: 本段累计已发字节数。"""
        progress = self.get_progress()
        if progress is None:
            self.log("Error: no response to 0xC3 segment check", "\033[1;31m")
            return False
        recv_len, recv_crc = progress
        local = crc32(self.firmware[:sent])
        if recv_len != sent or recv_crc != local:
            self.log(
                f"Segment check mismatch (device len={recv_len} crc=0x{recv_crc:08X}, "
                f"local len={sent} crc=0x{local:08X})",
                "\033[1;31m",
            )
            return False
        ack = self.commit()
        if ack != ACK_OK:
            self.log(f"Segment commit failed: {self.ack_name(ack)}", "\033[1;31m")
            return False
        return True

    def send_block_checked(self, index, offset, block, chunk_size):
        """发送单个 0xC4 块并重试（末块短块直发不补位），返回 True/False。
        index: 块序号；offset: 块在固件中的偏移；block: 块数据；chunk_size: 协商块大小。"""
        for attempt in range(1, 4):
            ack = self.send_block(block)
            if ack == ACK_OK:
                return True
            if ack is not None:
                self.log(
                    f"Block {index} NACK: {self.ack_name(ack)} ({attempt}/3)",
                    "\033[1;33m",
                )
            else:
                progress = self.get_progress()
                if progress and progress[0] >= offset + len(block):
                    self.log(
                        f"Block {index} ack lost but device has it; continuing",
                        "\033[1;33m",
                    )
                    return True
                if progress and progress[0] != offset:
                    self.log(
                        f"Transfer desync: device at {progress[0]}, expected {offset}",
                        "\033[1;31m",
                    )
                    return False
                self.log(f"Block {index} timeout ({attempt}/3)", "\033[1;33m")
            time.sleep(0.2)
        self.log(f"Block {index} failed after 3 attempts", "\033[1;31m")
        return False


if __name__ == "__main__":
    args = sys.argv[1:]

    port_arg = args[0] if args else "/etc/default/bms_daemon"
    if os.path.isfile(port_arg):
        with open(port_arg, "r") as f:
            for line in f:
                if "BMS_PORT=" in line:
                    port_arg = line.split("=", 1)[1].strip()
                    break

    bin_file = args[1] if len(args) > 1 else \
        "/opt/roboparty/lib/firmware/SCUD_Roboparty13S2P_V01.0.bin"

    ota = ScudOta(port_arg, bin_file)
    sys.exit(0 if ota.run() else 1)
