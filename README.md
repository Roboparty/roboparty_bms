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
./build_deb.sh
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

### 4. Service Management
```bash
sudo systemctl restart bms      # Restart service
sudo journalctl -u bms -f       # View real-time logs
```

## 🔄 OTA Firmware Update

1. Rename the new firmware to `firmware.bin` and place it in the `/opt/bms/` directory.
2. Execute the update command:
```bash
sudo systemctl start bms_ota
```
3. Monitor progress via `journalctl -u bms_ota -f`. The BMS will reset automatically after the update.
