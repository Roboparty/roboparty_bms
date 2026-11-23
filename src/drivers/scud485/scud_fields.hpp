// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

/**
 * @file scud_fields.hpp
 * @brief SCUD485 字段映射表与定点解码（表驱动）。
 * @details 每个通讯帧一张 constexpr 定义表（字段名/字节偏移/符号/精度/偏移），
 *          parse_fields() 按表把载荷解码进 FieldMap（std::unordered_map）。
 *          解码公式：实数 = raw × scale + value_off；多字节字段大端。
 *          电压/容量类 u16 无符号，电流/温度类补码（i16/i8）。
 *          新增帧 = 新增一张表，不改解码逻辑。
 */
#pragma once

#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>

#include "bms_driver.hpp"

namespace scud_bms {

enum class RawKind : uint8_t { U8, I8, U16, I16, BCD16 };

struct FieldSpec {
    const char* name;    // 查询键，如 "pplus_mv"
    uint16_t offset;     // 载荷内字节偏移（多字节大端）
    RawKind kind;        // 电压容量 U16 / 电流温度 I16、I8（补码）
    double scale;        // 精度
    double value_off;    // 偏移
};

using FieldMap = std::unordered_map<std::string, double>;

// ---- 0x61 BMS 通用信息，载荷 30 字节（BYTE0..BYTE29） ----
// TODO(精度/offset): 规格数值待提供，当前统一 scale=1, value_off=0，即 raw 值
inline constexpr FieldSpec kUserInfoFields[] = {
    {"bat_flag",      0, RawKind::U16, 1.0, 0.0}, // BAT FLAG
    {"chg_ma",        2, RawKind::I16, 1.0, 0.0}, // 充电电流 mA（补码；合并时充电为正）
    {"dis_ma",        4, RawKind::I16, 1.0, 0.0}, // 放电电流 mA（补码；合并时放电为负）
    {"chg_mv",        6, RawKind::U16, 1.0, 0.0}, // 充电电压 mV
    {"bplus_mv",      8, RawKind::U16, 1.0, 0.0}, // 电池 B+ 电压 mV
    {"pplus_mv",     10, RawKind::U16, 1.0, 0.0}, // 输出电压 P+ mV
    {"cell_mv_max",  12, RawKind::U16, 1.0, 0.0}, // 最高单体电压 mV
    {"cell_idx_max", 14, RawKind::U16, 1.0, 0.0}, // 最高单体序列
    {"cell_mv_min",  16, RawKind::U16, 1.0, 0.0}, // 最低单体电压 mV
    {"cell_idx_min", 18, RawKind::U16, 1.0, 0.0}, // 最低单体序列
    {"cap_mah",      20, RawKind::U16, 1.0, 0.0}, // 当前总容量 mAh
    {"remain_mah",   22, RawKind::U16, 1.0, 0.0}, // 剩余容量 mAh
    {"soc_pct",      24, RawKind::U16, 1.0, 0.0}, // SOC %
    {"cycles",       26, RawKind::U16, 1.0, 0.0}, // 循环次数
    {"vdif_mv",      28, RawKind::U16, 1.0, 0.0}, // 压差 mV
};

// ---- 0x31 电池信息汇总，载荷 61 字节（BYTE0..BYTE60） ----
// BYTE39 SOH 与 BYTE40..60 产品序列号（21 字节 ASCII，见 parse_sn）为固件 v75 新增。
// TODO(精度/offset): 同上，待规格数值提供后仅改本表常量
inline constexpr FieldSpec kSummaryFields[] = {
    {"cell1_mv",      0, RawKind::U16, 1.0, 0.0}, // 第 1 节电池电压 mV
    {"cell2_mv",      2, RawKind::U16, 1.0, 0.0},
    {"cell3_mv",      4, RawKind::U16, 1.0, 0.0},
    {"cell4_mv",      6, RawKind::U16, 1.0, 0.0},
    {"cell5_mv",      8, RawKind::U16, 1.0, 0.0},
    {"cell6_mv",     10, RawKind::U16, 1.0, 0.0},
    {"cell7_mv",     12, RawKind::U16, 1.0, 0.0},
    {"cell8_mv",     14, RawKind::U16, 1.0, 0.0},
    {"cell9_mv",     16, RawKind::U16, 1.0, 0.0},
    {"cell10_mv",    18, RawKind::U16, 1.0, 0.0},
    {"cell11_mv",    20, RawKind::U16, 1.0, 0.0},
    {"cell12_mv",    22, RawKind::U16, 1.0, 0.0},
    {"cell13_mv",    24, RawKind::U16, 1.0, 0.0}, // 第 13 节电池电压 mV
    {"mos_temp",     26, RawKind::I8,  1.0, 0.0}, // MOS 温度 ℃（补码）
    {"cell_temp1",   27, RawKind::I8,  1.0, 0.0}, // 电芯温度 1 ℃（补码）
    {"cell_temp2",   28, RawKind::I8,  1.0, 0.0}, // 电芯温度 2 ℃（补码）
    {"heater_temp1", 29, RawKind::I8,  1.0, 0.0}, // 加热膜 1 温度 ℃（补码）
    {"heater_temp2", 30, RawKind::I8,  1.0, 0.0}, // 加热膜 2 温度 ℃（补码）
    {"tdiff",        31, RawKind::I8,  1.0, 0.0}, // 温差 ℃（补码，按温度类）
    {"sw_version",   32, RawKind::BCD16, 1.0, 0.0}, // 软件版本号（BCD：0x0075 → 75）
    {"hw_version",   34, RawKind::BCD16, 1.0, 0.0}, // 硬件版本号（BCD：0x0100 → 100）
    {"alarm",        36, RawKind::U16, 1.0, 0.0}, // 告警
    {"protect2",     38, RawKind::U8,  1.0, 0.0}, // 2 级保护状态（1 字节）
    {"soh",          39, RawKind::U8,  1.0, 0.0}, // SOH %（固件 v75 起新增）
};

constexpr size_t kUserInfoFieldCount =
    sizeof(kUserInfoFields) / sizeof(kUserInfoFields[0]);
constexpr size_t kSummaryFieldCount =
    sizeof(kSummaryFields) / sizeof(kSummaryFields[0]);

// ---- 0x31 产品序列号（BYTE40..BYTE60，21 字节 ASCII，空格计入内容） ----
// 未烧录的包为空格，返回空串；由 read_serial_number() 单独交付（字符串
// 进不了纯 double 的 FieldMap）。
constexpr uint16_t kSnOffset = 40;
constexpr size_t kSnLen = 21;

inline std::string parse_sn(const uint8_t* payload, size_t len) {
    if (len < static_cast<size_t>(kSnOffset) + kSnLen) return {};
    std::string sn(reinterpret_cast<const char*>(payload + kSnOffset), kSnLen);
    while (!sn.empty() && (sn.back() == ' ' || sn.back() == '\0')) sn.pop_back();
    return sn;
}

// ---- 位定义表与展开 ----
// 0x31 告警（BYTE36-37 u16）与 0x61 BAT FLAG（BYTE0-1 u16）按位展开为
// 具名布尔字段（0.0/1.0），原值保留。
struct NamedBit {
    uint8_t bit;
    const char* name;
};

// 0x31 告警：Bit13-15 保留
inline constexpr NamedBit kAlarmBits[] = {
    {0,  "uv_alarm"},             // 欠压告警
    {1,  "ov_alarm"},             // 过压告警
    {2,  "chg_h_alarm_flag"},     // 充电高温告警
    {3,  "chg_l_alarm_flag"},     // 充电低温告警
    {4,  "dsg_h_alarm_flag"},     // 放电高温告警
    {5,  "dsg_l_alarm_flag"},     // 放电低温告警
    {6,  "mos_h_alarm_flag"},     // MOS 高温告警
    {7,  "cap_low_alarm"},        // 容量低告警
    {8,  "cell_volt_dif_alarm"},  // 电池压差告警
    {9,  "cell_temp_dif_alarm"},  // 电池温度压差告警
    {10, "cell_temp_h_alarm"},    // 电池温度高告警
    {11, "cell_temp_l_alarm"},    // 电池温度低告警
    {12, "afe_com_err"},          // Afe 通信失败
};

constexpr size_t kAlarmBitCount = sizeof(kAlarmBits) / sizeof(kAlarmBits[0]);

// 0x61 BAT FLAG：16 位全定义
inline constexpr NamedBit kBatFlagBits[] = {
    {0,  "dch_pre_err"},          // 电池包预放电错误
    {1,  "dch_pre_state"},        // 电池包预放电状态
    {2,  "dch_temp_l_protect"},   // 电池包放电低温保护
    {3,  "dch_temp_h_protect"},   // 电池包放电高温保护
    {4,  "mos_temp_h_protect"},   // MOS 高温保护电池包
    {5,  "chg_temp_h_protect"},   // 电池包充电高温保护
    {6,  "chg_temp_l_protect"},   // 电池包充电低温保护
    {7,  "afe_ov_flag"},          // 前端电池包过压保护
    {8,  "afe_uv_flag"},          // 前端电池包欠压保护
    {9,  "afe_scd_flag"},         // 前端电池包短路保护
    {10, "afe_ocd_flag"},         // 前端电池包放电过流保护
    {11, "afe_occ_flag"},         // 前端电池包充电过流保护
    {12, "sys_dch_state"},        // 电池包放电状态
    {13, "sys_chg_state"},        // 电池包充电状态
    {14, "sys_chg_on"},           // 电池包使能充电
    {15, "sys_dch_on"},           // 电池包使能放电
};

constexpr size_t kBatFlagBitCount =
    sizeof(kBatFlagBits) / sizeof(kBatFlagBits[0]);

// 把 map 中 word 键的 u16 原值展开为具名布尔字段（0.0/1.0）；原值保留；
// 无该键时不动。
inline void expand_bits(FieldMap& map, const char* word,
                        const NamedBit* bits, size_t count) {
    auto it = map.find(word);
    if (it == map.end()) return;
    const uint16_t value = static_cast<uint16_t>(it->second);
    for (size_t i = 0; i < count; ++i) {
        map[bits[i].name] = ((value >> bits[i].bit) & 1u) ? 1.0 : 0.0;
    }
}

inline double field_or(const FieldMap& map, const char* name, double fallback) {
    auto it = map.find(name);
    return it == map.end() ? fallback : it->second;
}

// Bridge: field map -> legacy BatteryStatus wire record. The main socket
// keeps the historical wire protocol and its units (V / A / Ah / 0..1
// fraction), so the legacy conversions apply here: the map currently holds
// raw wire numbers (scale=1/offset=0 pending the spec's fixed-point table).
// When that table lands and map values become real-world units, drop the
// /1000 and /100 coefficients below in the same change.
inline void fill_battery_status(const FieldMap& map, bms::BatteryStatus& status) {
    status.voltage = field_or(map, "pplus_mv", 0.0) / 1000.0;  // mV -> V
    // Legacy signed current: charging positive, discharging negative.
    const double chg = field_or(map, "chg_ma", 0.0);
    const double dis = field_or(map, "dis_ma", 0.0);
    status.current = ((chg > 0) ? chg : -dis) / 1000.0;        // mA -> A
    status.percentage = field_or(map, "soc_pct", 0.0) / 100.0; // % -> 0..1
    status.charge = field_or(map, "remain_mah", 0.0) / 1000.0; // mAh -> Ah
    status.capacity = field_or(map, "cap_mah", 0.0) / 1000.0;  // mAh -> Ah
    status.design_capacity = status.capacity; // no design capacity in spec
    // No single "temperature" field: report the warmer cell sensor.
    const double t1 = field_or(map, "cell_temp1", 0.0);
    const double t2 = field_or(map, "cell_temp2", 0.0);
    status.temperature = (t1 > t2) ? t1 : t2;
    status.max_cell_voltage = field_or(map, "cell_mv_max", 0.0) / 1000.0;
    status.min_cell_voltage = field_or(map, "cell_mv_min", 0.0) / 1000.0;

    const uint16_t flag = static_cast<uint16_t>(field_or(map, "bat_flag", 0.0));
    status.protect_status = flag;
    status.work_state = flag & 0x000F;
    // BAT FLAG Bit15 sys_dch_on: the pack enables discharge.
    status.power_on = (field_or(map, "sys_dch_on", 0.0) != 0.0) ? 1 : 0;

    status.sw_version = static_cast<uint16_t>(field_or(map, "sw_version", 0.0));
    status.hw_version = static_cast<uint16_t>(field_or(map, "hw_version", 0.0));
    status.cycles = static_cast<uint32_t>(field_or(map, "cycles", 0.0));
    status.soh = static_cast<uint16_t>(field_or(map, "soh", 0.0)); // %
    status.io_state = 0; // the protocol carries no IO bitmap
}

// raw → 实数 = raw × scale + value_off。越界字段跳过（防截断载荷读飞）。
inline void parse_fields(const uint8_t* payload, size_t len,
                         const FieldSpec* fields, size_t count,
                         FieldMap& out) {
    for (size_t i = 0; i < count; ++i) {
        const FieldSpec& f = fields[i];
        const size_t width =
            (f.kind == RawKind::U8 || f.kind == RawKind::I8) ? 1 : 2;
        if (static_cast<size_t>(f.offset) + width > len) continue;

        int32_t raw = 0;
        switch (f.kind) {
        case RawKind::U8:
            raw = payload[f.offset];
            break;
        case RawKind::I8:
            raw = static_cast<int8_t>(payload[f.offset]);
            break;
        case RawKind::U16:
            raw = (static_cast<uint16_t>(payload[f.offset]) << 8) |
                  payload[f.offset + 1];
            break;
        case RawKind::I16:
            raw = static_cast<int16_t>(
                (static_cast<uint16_t>(payload[f.offset]) << 8) |
                payload[f.offset + 1]);
            break;
        case RawKind::BCD16: {
            // 版本字按 BCD 解读：4 个十六进制位即十进制数值（0x0075 → 75）
            const int b0 = payload[f.offset];
            const int b1 = payload[f.offset + 1];
            raw = (b0 >> 4) * 1000 + (b0 & 0x0F) * 100 +
                  (b1 >> 4) * 10 + (b1 & 0x0F);
            break;
        }
        }
        out[f.name] = static_cast<double>(raw) * f.scale + f.value_off;
    }
}

// ---- 线协议：字段快照记录（Unix socket，同机原生字节序） ----
//   u32 payload_len | payload
//   payload: u16 count | count × (u8 name_len | name | f64 value)
constexpr uint32_t kMaxFieldRecordBytes = 16384;

inline std::string serialize_field_map(const FieldMap& map) {
    std::string payload;
    const uint16_t count = static_cast<uint16_t>(map.size());
    payload.append(reinterpret_cast<const char*>(&count), sizeof(count));
    for (const auto& kv : map) {
        const uint8_t name_len = static_cast<uint8_t>(kv.first.size());
        payload.push_back(static_cast<char>(name_len));
        payload.append(kv.first);
        const double v = kv.second;
        payload.append(reinterpret_cast<const char*>(&v), sizeof(v));
    }
    std::string record;
    const uint32_t len = static_cast<uint32_t>(payload.size());
    record.append(reinterpret_cast<const char*>(&len), sizeof(len));
    record += payload;
    return record;
}

// 解析一条记录的 payload（不含 u32 长度前缀）。失败返回 false（out 不动）。
inline bool deserialize_field_map(const uint8_t* payload, size_t len,
                                  FieldMap& out) {
    if (len < 2) return false;
    uint16_t count = 0;
    std::memcpy(&count, payload, sizeof(count));
    size_t pos = sizeof(count);

    FieldMap map;
    for (uint16_t i = 0; i < count; ++i) {
        if (pos + 1 > len) return false;
        const uint8_t name_len = payload[pos++];
        if (pos + name_len + sizeof(double) > len) return false;
        std::string name(reinterpret_cast<const char*>(payload + pos), name_len);
        pos += name_len;
        double value = 0.0;
        std::memcpy(&value, payload + pos, sizeof(value));
        pos += sizeof(value);
        map[std::move(name)] = value;
    }
    if (pos != len) return false; // 尾部多余字节 = 记录错位
    out = std::move(map);
    return true;
}

} // namespace scud_bms
