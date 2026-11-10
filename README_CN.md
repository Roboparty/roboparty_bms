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

> **`BMS_TYPE=SCUD485` 的 `BAUD_RATE` 必须是 `115200`。**

### 4. 服务管理
```bash
sudo systemctl restart bms      # 重启服务
sudo journalctl -u bms -f       # 查看实时运行日志
```

> **485 读取在服务启动 10 秒后才开始。** `bms.service` 里有 `ExecStartPre=/bin/sleep 10`，
> 因为电池包上电后需要一段时间才能在总线上应答。每次启动都要等这 10 秒，`systemctl restart bms`
> 也不例外；等待期间 `/tmp/bms.sock` 还不存在。所以开机后头 10~12 秒 `is_connected()` 返回
> `false` 是预期行为，不是故障。

## UART 转发

`bms_daemon` 可以把从电池总线上读到的每一个字节同时镜像到第二个串口。这个功能取代了
`robopi-addon` 里的 `robopi-uart-bridge` —— 后者是另一个进程在读同一个串口。**两个读者读同一个
tty 会互相抢走字节**，结果是 daemon 反复记录 Modbus 读失败，下游端口也只能拿到残缺的数据流。
现在只有一个读者，它在自己读的同时把数据分流出去，下游能拿到完整流，而 BMS 侧不受任何影响。

在 `/etc/default/bms_daemon` 中配置：

| 配置项 | 默认值 | 含义 |
|---|---|---|
| `FORWARD_ENABLE` | `1` | 总开关 |
| `FORWARD_PORT_A` | `/dev/ttyS3` | 源端口，只读 |
| `FORWARD_PORT_B` | `/dev/ttyS7` | 目的端口，只写 |
| `FORWARD_BAUD` | `115200` | 目的端口速率 |

- 当 `FORWARD_PORT_A` 就是 BMS 自己的端口时，daemon 直接分流自己读到的数据，**绝不会把该端口
  打开第二次**。当它指向别的端口时（BMS 走 CAN，或走另一个 UART），daemon 会单独打开该端口并用
  独立线程抽取。端口名按设备解析，因此 `/dev/serial/by-id/...` 这类指向 BMS 端口的软链也能被正确
  识别为同一个端口，不会退化成第二个读者。
- 只转发 daemon 真正读到的字节，也就是电池的应答，**不包含 daemon 自己发出的查询**。原来的桥接
  同样看不到这些发送内容。
- 目的端口是惰性打开并带重试的，`/dev/ttyS7` 不存在**不会**阻塞 BMS 工作。转发只是监视用途，
  绝不允许拖慢 Modbus 读循环；如果对端不再接收，字节会被丢弃并在 journal 中报告。

关闭转发：

```bash
sudo sed -i 's/^FORWARD_ENABLE=.*/FORWARD_ENABLE=0/' /etc/default/bms_daemon
sudo systemctl restart bms
```

## 数据新鲜度（SCUD485）

SCUD485 客户端把"链路通"和"电池在应答"分开：`bms_daemon` 每秒做一轮 0x61+0x31 查询，全部成功才
广播一条完整快照、失败就什么都不发——每条上线的记录都对自身全部字段（含 power_on）负责，
绝不混入上一轮的旧值；socket 常开而总线断掉因此是可识别的。

- `is_connected()` —— socket 已连接**且** 3 秒内收到过完整快照。总线断掉时返回 `false`；
  刚重连、尚未收到记录时同样返回 `false`。使用任何数值接口前先判它。
- `is_power_on()` —— 数据过期时返回 `false`，绝不返回最后一次的值。

数值类接口（`get_voltage()`、`get_percentage()` 等）刻意继续返回最后收到的快照，避免短暂
通信抖动被误判为"电池消失"。TWS 与 GF485 客户端未改动。

## 按名查询（SCUD485）

SCUD485 走**双协议并存**：

- **主 socket（`/tmp/bms.sock`）**：与旧协议完全一致——广播 `BatteryStatus` 定长结构体
  （单位语义 V/Ah/0..1 分数不变），旧客户端零影响；
- **副 socket（`/tmp/bms.sock.fields`）**：广播序列化字段映射（FieldMap），daemon 把
  0x61/0x31 两帧按 `src/drivers/scud485/scud_fields.hpp` 的定义表解码成"字段名 → 数值"
  后整体发布，客户端用 `get_field(name)` 按名取值（Python：`bms_py.BmsDriver.get_field(name)`）。

```cpp
auto bms = BmsDriver::create_bms("SCUD485");
double pplus_mv = bms->get_field("pplus_mv");   // 输出电压 P+
double soc      = bms->get_field("soc_pct");    // SOC
bool   uv_alarm = bms->get_field("uv_alarm") != 0.0;  // 欠压告警位
if (std::isnan(bms->get_field("nope"))) { /* 字段不存在或暂无数据 */ }
```

- **无异常**：字段不存在（或副 socket 尚未连上）时返回 **NaN**，用 `std::isnan()` /
  `math.isnan()` 判断，绝不抛异常；旧接口（`get_voltage()` 等）语义不变；
- **解码公式**：`实数 = raw × 精度 + 偏移`；当前所有字段精度=1、偏移=0（即帧上 raw 值），
  待规格定点参数确定后仅需修改 `scud_fields.hpp` 中的表常量；
- **符号**：电压/容量类无符号（u16），电流/温度类补码（i16/i8）；
- **电流**：`chg_ma`/`dis_ma` 分开提供（`get_field` 取 raw mA）；主协议的
  `get_current()` 合并为有符号值（**充电为正、放电为负**，单位 A）；
- **告警**：0x31 的告警字按位展开为具名布尔字段（0.0/1.0）——`uv_alarm`（欠压）、
  `ov_alarm`（过压）、`chg_h_alarm_flag`/`chg_l_alarm_flag`（充高/低温）、
  `dsg_h_alarm_flag`/`dsg_l_alarm_flag`（放高/低温）、`mos_h_alarm_flag`（MOS高温）、
  `cap_low_alarm`（容量低）、`cell_volt_dif_alarm`/`cell_temp_dif_alarm`（压差/温度压差）、
  `cell_temp_h_alarm`/`cell_temp_l_alarm`（电池温度高/低）、`afe_com_err`（Afe通信失败），
  原值仍可通过 `get_field("alarm")` 取得；
- **BAT FLAG**：0x61 的状态字同样按位展开——`sys_dch_on`（Bit15 使能放电，即
  `is_power_on()` 的依据；注：旧版误取 Bit0 预放电错误位，已按规格修正）、
  `sys_chg_on`（Bit14 使能充电）、`sys_dch_state`/`sys_chg_state`
  （Bit12/13 放/充电状态）、`afe_occ_flag`/`afe_ocd_flag`/`afe_scd_flag`/`afe_uv_flag`/
  `afe_ov_flag`（Bit7-11 前端充/放电过流、短路、欠压、过压保护）、
  `chg_temp_h_protect`/`chg_temp_l_protect`（Bit5/6 充电高/低温保护）、
  `dch_temp_h_protect`/`dch_temp_l_protect`（Bit2/3 放电高/低温保护）、
  `mos_temp_h_protect`（Bit4 MOS高温保护）、`dch_pre_state`/`dch_pre_err`
  （Bit1/0 预放电状态/错误），原值仍可通过 `get_field("bat_flag")` 取得；
- **全部字段**：0x61——`bat_flag`、`chg_ma`、`dis_ma`、`chg_mv`、`bplus_mv`、`pplus_mv`、
  `cell_mv_max`、`cell_idx_max`、`cell_mv_min`、`cell_idx_min`、`cap_mah`、`remain_mah`、
  `soc_pct`、`cycles`、`vdif_mv`；0x31——`cell1_mv`..`cell13_mv`、`mos_temp`、`cell_temp1`、
  `cell_temp2`、`heater_temp1`、`heater_temp2`、`tdiff`、`sw_version`、`hw_version`、
  `alarm`、`protect2`（及上文全部位字段）。

## 🔄 OTA 固件升级

1. 将新固件放到随包服务的默认路径
   `${ROBOOS_PREFIX:-/opt/roboparty}/lib/firmware/TWS_LB-13S2P_APP_v008_20260804-00.bin`；
   也可以直接用脚本指定任意固件文件：
   ```bash
   sudo ${ROBOOS_PREFIX:-/opt/roboparty}/bin/bms_ota.py /etc/default/bms_daemon /path/to/firmware.bin
   ```
2. 执行升级指令：
   ```bash
   sudo systemctl start bms_ota
   ```
3. 通过 `journalctl -u bms_ota -f` 监控升级进度。升级完成后 BMS 会自动复位。
