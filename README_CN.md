# TWS BMS 守护进程 (bms_daemon)

这是一个专为 TWS (明美新能源) 动力电池组设计的工业级 C++ 守护进程。它负责与硬件进行底层的 Modbus RTU 通讯，并通过 Unix Domain Socket (`/tmp/bms.sock`) 向多个客户端实时广播电池状态。

## 🌟 核心特性

- **高性能通讯**：基于 C++ 编写，采用非阻塞 `poll` 机制，支持 300ms 响应超时控制。
- **高鲁棒性**：
    - **自愈重连**：连续 5 次读取失败会自动重置并重新初始化串口。
    - **总线保护**：指令间强制 50ms 延时，防止 Modbus 总线拥堵。
- **数据广播**：使用轻量级 Unix Domain Socket，支持多个客户端（如 ROS 节点、Python 脚本）同时订阅。
- **静态信息提取**：启动时自动打印电池固件版本、硬件版本、健康度 (SOH) 及循环次数。
- **内置 OTA 支持**：集成完整的固件在线升级协议和自动化升级服务。
- **专业打包**：支持一键生成 `.deb` 安装包，适配 Armbian 等 Linux 发行版，支持交叉编译。

## 📂 目录结构

- `src/`：核心源代码（守护进程逻辑与协议实现）。
- `include/`：头文件（包含公共数据结构 `bms_status.h`）。
- `config/`：默认配置文件模板。
- `service/`：Systemd 服务定义文件。
- `scripts/`：OTA 升级 Python 脚本。
- `udev/`：USB 串口设备权限与别名规则。

## 🛠️ 安装与运行

### 1. 编译并打包
在目标设备（如 OrangePi）上运行：
```bash
dpkg-buildpackage
```

### 2. 安装
```bash
sudo dpkg -i bms-daemon_1.1.3_arm64.deb
```

### 3. 配置端口
安装后，编辑配置文件以匹配您的实际串口（如 `/dev/ttyUSB1`）：
```bash
sudo nano /etc/default/bms_daemon
```

> **`BMS_TYPE=SCUD485` 必须配 `BAUD_RATE=19200`。** 485 协议文档第 2 节固定为 19200 波特率 /
> 8 数据位 / 无校验 / 1 停止位，而包内默认是 `115200`，因此改类型的同时必须改波特率。
> 另外 `SerialPort::open()` 只认 9600/19200/115200，其余值会**静默回落到 9600**，写错一个数字
> 表现出来就是"电池完全不应答"（连续 5 次读失败后重开串口），而不是配置错误。

### 4. 服务管理
```bash
sudo systemctl restart bms      # 重启服务
sudo journalctl -u bms -f       # 查看实时运行日志
```

> **485 读取在服务启动 10 秒后才开始。** `bms.service` 里有 `ExecStartPre=/bin/sleep 10`，
> 因为电池包上电后需要一段时间才能在总线上应答。每次启动都要等这 10 秒，`systemctl restart bms`
> 也不例外；等待期间 `/tmp/bms.sock` 还不存在。所以开机后头 10~12 秒 `is_connected()` 返回
> `false` 是预期行为，不是故障。

## 数据新鲜度（SCUD485）

SCUD485 客户端把"链路通"和"电池在应答"分开：`bms_daemon` 每秒只做一次 0x61 查询，成功就
广播一条完整快照、失败就什么都不发——每条上线的记录都对自身全部字段（含 power_on）负责，
绝不混入上一轮的旧值；socket 常开而总线断掉因此是可识别的。

- `is_connected()` —— socket 已连接**且** 3 秒内收到过完整快照。总线断掉时返回 `false`；
  刚重连、尚未收到记录时同样返回 `false`。使用任何数值接口前先判它。
- `is_power_on()` —— 数据过期时返回 `false`，绝不返回最后一次的值。

数值类接口（`get_voltage()`、`get_percentage()` 等）刻意继续返回最后收到的快照，避免短暂
通信抖动被误判为"电池消失"。TWS 与 GF485 客户端未改动。

## 🔄 OTA 固件升级

1. 将新固件命名为 `firmware.bin` 放入 `/opt/bms/` 目录。
2. 执行升级指令：
```bash
sudo systemctl start bms_ota
```
3. 通过 `journalctl -u bms_ota -f` 监控升级进度。升级完成后 BMS 会自动复位。
