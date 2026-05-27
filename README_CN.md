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
./build_deb.sh
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

### 4. 服务管理
```bash
sudo systemctl restart bms      # 重启服务
sudo journalctl -u bms -f       # 查看实时运行日志
```

## 🔄 OTA 固件升级

1. 将新固件命名为 `firmware.bin` 放入 `/opt/bms/` 目录。
2. 执行升级指令：
```bash
sudo systemctl start bms_ota
```
3. 通过 `journalctl -u bms_ota -f` 监控升级进度。升级完成后 BMS 会自动复位。
