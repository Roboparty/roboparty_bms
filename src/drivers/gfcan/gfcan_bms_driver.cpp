// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "gfcan_bms_driver.hpp"

#include <cstring>
#include <functional>

#include "protocol/can/socket_can.hpp"

namespace bms {

static constexpr uint32_t CID_BMS_0  = 0x18F00003;
static constexpr uint32_t CID_BMS_1  = 0x18F00103;
static constexpr uint32_t CID_BMS_2  = 0x18F00203;
static constexpr uint32_t CID_BMS_3  = 0x18F00303;
static constexpr uint32_t CID_BMS_4  = 0x18F00403;
static constexpr uint32_t CID_BMS_5  = 0x18F00503;
static constexpr uint32_t CID_BMS_6  = 0x18F00603;
static constexpr uint32_t CID_BMS_7  = 0x18F00703;
static constexpr uint32_t CID_BMS_8  = 0x18F00803;
static constexpr uint32_t CID_BMS_9  = 0x18F00903;
static constexpr uint32_t CID_BMS_10 = 0x18F00A03;
static constexpr uint32_t CID_BMS_11 = 0x18F00B03;
static constexpr uint32_t CID_BMS_21 = 0x18F01503;
static constexpr uint32_t CID_BMS_22 = 0x18F01603;

GfCanBmsDriver::GfCanBmsDriver(const std::string& can_iface)
    : BmsDriver(), can_iface_(can_iface) {
    std::memset(&cached_, 0, sizeof(cached_));

    can_ = SocketCAN::get_instance(can_iface_);

    can_->set_id_extractor([](const can_frame& f) -> CanId {
        return f.can_id & CAN_EFF_MASK;
    });

    auto cb = std::bind(&GfCanBmsDriver::can_callback, this, std::placeholders::_1);

    can_->add_callback(CID_BMS_0,  cb);
    can_->add_callback(CID_BMS_1,  cb);
    can_->add_callback(CID_BMS_2,  cb);
    can_->add_callback(CID_BMS_3,  cb);
    can_->add_callback(CID_BMS_4,  cb);
    can_->add_callback(CID_BMS_5,  cb);
    can_->add_callback(CID_BMS_6,  cb);
    can_->add_callback(CID_BMS_7,  cb);
    can_->add_callback(CID_BMS_8,  cb);
    can_->add_callback(CID_BMS_9,  cb);
    can_->add_callback(CID_BMS_10, cb);
    can_->add_callback(CID_BMS_11, cb);
    can_->add_callback(CID_BMS_21, cb);
    can_->add_callback(CID_BMS_22, cb);

    connected_ = true;
    logger_->info("GF CAN BMS driver started, iface={}", can_iface_);
}

GfCanBmsDriver::~GfCanBmsDriver() {
    connected_ = false;
    if (can_) {
        can_->remove_callback(CID_BMS_0);
        can_->remove_callback(CID_BMS_1);
        can_->remove_callback(CID_BMS_2);
        can_->remove_callback(CID_BMS_3);
        can_->remove_callback(CID_BMS_4);
        can_->remove_callback(CID_BMS_5);
        can_->remove_callback(CID_BMS_6);
        can_->remove_callback(CID_BMS_7);
        can_->remove_callback(CID_BMS_8);
        can_->remove_callback(CID_BMS_9);
        can_->remove_callback(CID_BMS_10);
        can_->remove_callback(CID_BMS_11);
        can_->remove_callback(CID_BMS_21);
        can_->remove_callback(CID_BMS_22);
    }
}

void GfCanBmsDriver::can_callback(const can_frame& frame) {
    std::unique_lock lock(mutex_);

    uint32_t id = frame.can_id & CAN_EFF_MASK;

    switch (id) {
    case CID_BMS_0:  parse_bms0(frame);  break;
    case CID_BMS_1:  parse_bms1(frame);  break;
    case CID_BMS_2:  parse_bms2(frame);  break;
    case CID_BMS_3:  parse_bms3(frame);  break;
    case CID_BMS_4:  parse_bms4(frame);  break;
    case CID_BMS_5:  parse_bms5(frame);  break;
    case CID_BMS_6:  parse_bms6(frame);  break;
    case CID_BMS_7:  parse_bms7(frame);  break;
    case CID_BMS_8:  parse_bms8(frame);  break;
    case CID_BMS_9:  parse_bms9(frame);  break;
    case CID_BMS_10: parse_bms10(frame); break;
    case CID_BMS_11: parse_bms11(frame); break;
    case CID_BMS_21: parse_bms21(frame); break;
    case CID_BMS_22: parse_bms22(frame); break;
    default: break;
    }
}

/* BMS_0: Fault/Protect Status */
void GfCanBmsDriver::parse_bms0(const can_frame& f) {
    uint32_t fault_code = u8(f,2) | (u8(f,3) << 8) | ((u8(f,4) & 0x3) << 16);
    cached_.protect_status = u8(f,0) | (u8(f,1) << 8)
                           | (fault_code << 16)
                           | (static_cast<uint32_t>((u8(f,4) >> 2) & 0x7) << 29);
}

/* BMS_1: Error/Bitmask Status */
void GfCanBmsDriver::parse_bms1(const can_frame& f) {
    cached_.work_state = (cached_.work_state & 0xFF00) | u16_le(f, 0);
}

/* BMS_2: SOC, SOH, Voltage, Current */
void GfCanBmsDriver::parse_bms2(const can_frame& f) {
    cached_.percentage = u8(f, 0);
    cached_.soh        = u8(f, 1);
    cached_.voltage    = u16_le(f, 2) * 0.1;
    cached_.current    = (static_cast<int16_t>(u16_le(f, 4)) - 10000) * 0.1;
}

/* BMS_3: Current ratings – no key mapped fields */

/* BMS_4: Temperatures */
void GfCanBmsDriver::parse_bms4(const can_frame& f) {
    cached_.temperature = u8(f, 3) - 40;
}

/* BMS_5: Cell Max/Min Voltage */
void GfCanBmsDriver::parse_bms5(const can_frame& f) {
    cached_.max_cell_voltage = u16_le(f, 0) * 0.001;
    cached_.min_cell_voltage = u16_le(f, 4) * 0.001;
}

/* BMS_9: Software / Hardware Version */
void GfCanBmsDriver::parse_bms9(const can_frame& f) {
    cached_.sw_version = u8(f, 0) | (u8(f, 1) << 8);
    cached_.hw_version = u8(f, 3) | (u8(f, 4) << 8);
}

/* BMS_10: Battery Pack State / MOS Control */
void GfCanBmsDriver::parse_bms10(const can_frame& f) {
    cached_.work_state = (cached_.work_state & 0x00FF) | ((u8(f, 0) & 0x3F) << 8);
}

/* BMS_11: MOS Status / Charger State */
void GfCanBmsDriver::parse_bms11(const can_frame& f) {
    cached_.work_state = (cached_.work_state & 0x3FFF) | ((u8(f, 1) & 0x07) << 14);
}

/* BMS_21: Design / Full / Current Capacity */
void GfCanBmsDriver::parse_bms21(const can_frame& f) {
    cached_.design_capacity = u16_le(f, 0) * 0.01;
    cached_.capacity        = u16_le(f, 2) * 0.01;
    cached_.charge          = u16_le(f, 6) * 0.01;
}

/* BMS_22: Total Charge/Discharge Capacity, Cycles */
void GfCanBmsDriver::parse_bms22(const can_frame& f) {
    cached_.cycles = u16_le(f, 4);
}

/* BMS_6 / BMS_7 / BMS_8: Serial Number (24 ASCII bytes) */
void GfCanBmsDriver::parse_bms6(const can_frame& f) {
    for (int i = 0; i < 8 && i < 32; ++i)
        cached_.serial_number[i] = static_cast<char>(f.data[i]);
}

void GfCanBmsDriver::parse_bms7(const can_frame& f) {
    for (int i = 0; i < 8 && (i + 8) < 32; ++i)
        cached_.serial_number[i + 8] = static_cast<char>(f.data[i]);
}

void GfCanBmsDriver::parse_bms8(const can_frame& f) {
    for (int i = 0; i < 8 && (i + 16) < 32; ++i)
        cached_.serial_number[i + 16] = static_cast<char>(f.data[i]);
    cached_.serial_number[24] = '\0';
}

/* BMS_3: empty handler (charge/discharge ratings – not cached) */
void GfCanBmsDriver::parse_bms3(const can_frame&) {}

double    GfCanBmsDriver::get_voltage()          const { std::shared_lock l(mutex_); return cached_.voltage; }
double    GfCanBmsDriver::get_current()          const { std::shared_lock l(mutex_); return cached_.current; }
double    GfCanBmsDriver::get_temperature()      const { std::shared_lock l(mutex_); return cached_.temperature; }
double    GfCanBmsDriver::get_percentage()       const { std::shared_lock l(mutex_); return cached_.percentage; }
double    GfCanBmsDriver::get_charge()           const { std::shared_lock l(mutex_); return cached_.charge; }
double    GfCanBmsDriver::get_capacity()         const { std::shared_lock l(mutex_); return cached_.capacity; }
double    GfCanBmsDriver::get_design_capacity()  const { std::shared_lock l(mutex_); return cached_.design_capacity; }
uint32_t  GfCanBmsDriver::get_protect_status()   const { std::shared_lock l(mutex_); return cached_.protect_status; }
uint16_t  GfCanBmsDriver::get_work_state()        const { std::shared_lock l(mutex_); return cached_.work_state; }
double    GfCanBmsDriver::get_max_cell_voltage() const { std::shared_lock l(mutex_); return cached_.max_cell_voltage; }
double    GfCanBmsDriver::get_min_cell_voltage() const { std::shared_lock l(mutex_); return cached_.min_cell_voltage; }
uint16_t  GfCanBmsDriver::get_soh()              const { std::shared_lock l(mutex_); return cached_.soh; }
uint32_t  GfCanBmsDriver::get_cycles()           const { std::shared_lock l(mutex_); return cached_.cycles; }
bool      GfCanBmsDriver::is_connected()         const { return connected_.load(); }

} // namespace bms
