# TWS BMS Daemon (bms_daemon)

An industrial-grade C++ daemon designed for TWS power battery packs. It handles low-level Modbus RTU communication with the hardware and broadcasts battery status to multiple clients via a Unix Domain Socket (`/tmp/bms.sock`).

## 🌟 Key Features

- **High-Performance Communication**: Written in C++ with a non-blocking `poll` mechanism, supporting 300ms response timeout control.
- **High Robustness**:
    - **Self-healing Reconnection**: Automatically resets and re-initializes the serial port after 5 consecutive read failures.
    - **Bus Protection**: Enforces a 50ms delay between commands to prevent Modbus bus congestion.
- **Data Broadcasting**: Uses lightweight Unix Domain Sockets, allowing multiple clients (ROS nodes, Python scripts) to subscribe simultaneously.
- **Static Info Extraction**: Automatically logs firmware/hardware versions, Health (SOH), and cycle counts upon startup.
- **Built-in OTA Support**: Integrated firmware over-the-air update protocol and automated update service.
- **Professional Packaging**: Supports one-click `.deb` package generation, optimized for Armbian and other Linux distros, with cross-compilation support.

## 📂 Directory Structure

- `src/`: Core source code (Daemon logic and protocol implementation).
- `include/`: Header files (Common data structure `bms_status.h`).
- `config/`: Default configuration templates.
- `service/`: Systemd service definition files.
- `scripts/`: OTA update Python scripts.
- `udev/`: USB serial device permission and alias rules.

## 🛠️ Installation & Running

### 1. Build and Package
Run on the target device (e.g., OrangePi):
```bash
dpkg-buildpackage
```

### 2. Install
```bash
sudo dpkg -i bms-daemon_1.1.3_arm64.deb
```

### 3. Configure Port
After installation, edit the config file to match your actual serial port (e.g., `/dev/ttyUSB1`):
```bash
sudo nano /etc/default/bms_daemon
```

> **`BMS_TYPE=SCUD485` requires `BAUD_RATE=19200`.** The 485 spec (sec. 2 Serial Attributes) fixes
> 19200 baud / 8 data bits / no parity / 1 stop bit, while the shipped default is `115200`, so the
> type must be changed together with the baud rate. Note that `SerialPort::open()` only knows
> 9600/19200/115200 and silently falls back to 9600 for any other value — a typo then looks like a
> dead battery (5 read failures, port reopen) instead of a configuration error.

### 4. Service Management
```bash
sudo systemctl restart bms      # Restart service
sudo journalctl -u bms -f       # View real-time logs
```

> **485 reads start 10 s after the service launches.** `bms.service` runs `ExecStartPre=/bin/sleep 10`
> because the pack needs time after power-on before it answers on the bus. Every start pays the same
> 10 s, including `systemctl restart bms`, and `/tmp/bms.sock` does not exist until the wait is over —
> `is_connected()` reporting `false` for the first ~10–12 s of a boot is expected, not a fault.

## Data Freshness (SCUD485)

The SCUD485 client separates "the transport is up" from "the battery is answering".
`bms_daemon` polls the pack with one 0x61 query per second and publishes either a complete
fresh snapshot or nothing at all: a record on the wire always vouches for every field it
carries (power_on included) and never mixes in values from an older round, so a socket that
stays open while the bus is dead is detectable.

- `is_connected()` — socket open **and** a complete snapshot received within 3 s. A live
  socket with a dead bus reports `false`, and so does a socket that has just reconnected and
  not yet seen a record. Check this before acting on any numeric getter.
- `is_power_on()` — `false` while the data is stale, never the last known value.

Numeric getters (`get_voltage()`, `get_percentage()`, ...) keep returning the last received
snapshot on purpose, so a short bus dropout does not look like a missing battery. TWS and
GF485 clients are unchanged.

## 🔄 OTA Firmware Update

1. Rename the new firmware to `firmware.bin` and place it in the `/opt/bms/` directory.
2. Execute the update command:
```bash
sudo systemctl start bms_ota
```
3. Monitor progress via `journalctl -u bms_ota -f`. The BMS will reset automatically after the update.
