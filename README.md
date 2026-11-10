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

> **`BMS_TYPE=SCUD485` requires `BAUD_RATE=115200`.**

### 4. Service Management
```bash
sudo systemctl restart bms      # Restart service
sudo journalctl -u bms -f       # View real-time logs
```

> **485 reads start 10 s after the service launches.** `bms.service` runs `ExecStartPre=/bin/sleep 10`
> because the pack needs time after power-on before it answers on the bus. Every start pays the same
> 10 s, including `systemctl restart bms`, and `/tmp/bms.sock` does not exist until the wait is over —
> `is_connected()` reporting `false` for the first ~10–12 s of a boot is expected, not a fault.

## UART Forwarding

`bms_daemon` can mirror every byte it reads from the BMS onto a second UART. This replaces
`robopi-uart-bridge` from `robopi-addon`, which ran as a separate process on the same port: two
readers on one tty steal bytes from each other, so the daemon logged intermittent Modbus read
failures and the downstream port only ever received a partial stream. One reader tapping its own
feed hands the downstream port the complete stream without costing the BMS anything.

Configured in `/etc/default/bms_daemon`:

| Key | Default | Meaning |
|---|---|---|
| `FORWARD_ENABLE` | `1` | Master switch |
| `FORWARD_PORT_A` | `/dev/ttyS3` | Source, read only |
| `FORWARD_PORT_B` | `/dev/ttyS7` | Destination, written only |
| `FORWARD_BAUD` | `115200` | Destination line speed |

- When `FORWARD_PORT_A` is the BMS port itself, the daemon taps its own reads; it never opens the
  port a second time. When it names a different port (BMS on CAN, or on another UART) the daemon
  opens that port and pumps it on its own thread. Names are resolved by device, so a symlink such
  as `/dev/serial/by-id/...` pointing at the BMS port is still recognised as the same port.
- Only bytes the daemon actually reads are forwarded: the BMS's replies, not the daemon's own
  queries. The old bridge never saw those transmissions either.
- The destination is opened lazily and retried, so a missing `/dev/ttyS7` never holds up the BMS.
  Forwarding is a monitor feed and is never allowed to slow the Modbus read loop down; if the
  destination stops accepting, bytes are dropped and reported in the journal.
- `FORWARD_BAUD` is independent of `BAUD_RATE`. Both default to 115200, but when the bus runs at
  another rate set `FORWARD_BAUD` to match, otherwise the forwarded stream is garbled while still
  looking plausible.

To turn it off:

```bash
sudo sed -i 's/^FORWARD_ENABLE=.*/FORWARD_ENABLE=0/' /etc/default/bms_daemon
sudo systemctl restart bms
```

## Data Freshness (SCUD485)

The SCUD485 client separates "the transport is up" from "the battery is answering".
`bms_daemon` polls the pack with one 0x61+0x31 round per second and publishes either a complete
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

## By-Name Query (SCUD485)

SCUD485 runs **dual protocols side by side**:

- **main socket (`/tmp/bms.sock`)**: the legacy wire protocol, unchanged -- fixed-size
  `BatteryStatus` records in the historical units (V / Ah / 0..1 fraction). Existing clients
  are unaffected.
- **fields socket (`/tmp/bms.sock.fields`)**: serialized field-map records. The daemon decodes
  the 0x61/0x31 frames per the definition tables in `src/drivers/scud485/scud_fields.hpp` and
  clients fetch values with `get_field(name)` (Python: `bms_py.BmsDriver.get_field(name)`).

`get_field` never throws: **NaN means "no such field" (or no data yet)** -- check with
`std::isnan()` / `math.isnan()`. Decode is `raw × scale + value_off`; every field currently
uses scale=1 / offset=0 (raw wire values) until the spec's fixed-point parameters land -- only
the table constants change then. Voltage/capacity fields are unsigned, current/temperature
two's complement; `get_current()` merges `chg_ma`/`dis_ma` as charging positive, discharging
negative. The 0x31 alarm word is expanded into named boolean bits (`uv_alarm`, `ov_alarm`,
`chg_h_alarm_flag`, `chg_l_alarm_flag`, `dsg_h_alarm_flag`, `dsg_l_alarm_flag`,
`mos_h_alarm_flag`, `cap_low_alarm`, `cell_volt_dif_alarm`, `cell_temp_dif_alarm`,
`cell_temp_h_alarm`, `cell_temp_l_alarm`, `afe_com_err`); the raw word stays under `alarm`.
BAT FLAG expands likewise (`sys_dch_on` at Bit15 is what `is_power_on()` reports -- the old
build mistakenly used Bit0). Full key list in the Chinese README section above.

## 🔄 OTA Firmware Update

1. Place the new firmware package at
   `${ROBOOS_PREFIX:-/opt/roboparty}/lib/firmware/TWS_LB-13S2P_APP_v008_20260804-00.bin`
   (the path used by the packaged service), or run the script directly with a
   custom file:
   ```bash
   sudo ${ROBOOS_PREFIX:-/opt/roboparty}/bin/bms_ota.py /etc/default/bms_daemon /path/to/firmware.bin
   ```
2. Execute the update command:
   ```bash
   sudo systemctl start bms_ota
   ```
3. Monitor progress via `journalctl -u bms_ota -f`. The BMS will reset automatically after the update.
