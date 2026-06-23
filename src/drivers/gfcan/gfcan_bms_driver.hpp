// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#pragma once

#include <linux/can.h>

#include <atomic>
#include <memory>
#include <shared_mutex>
#include <string>

#include "bms_driver.hpp"
#include "protocol/can/socket_can.hpp"

namespace bms {

class GfCanBmsDriver : public BmsDriver {
public:
    explicit GfCanBmsDriver(const std::string& can_iface);
    ~GfCanBmsDriver() override;

    double get_voltage() const override;
    double get_current() const override;
    double get_temperature() const override;
    double get_percentage() const override;
    double get_charge() const override;
    double get_capacity() const override;
    double get_design_capacity() const override;
    uint32_t get_protect_status() const override;
    uint16_t get_work_state() const override;
    double get_max_cell_voltage() const override;
    double get_min_cell_voltage() const override;
    uint16_t get_soh() const override;
    uint32_t get_cycles() const override;
    bool is_connected() const override;

private:
    void can_callback(const can_frame& frame);

    void parse_bms0(const can_frame& frame);
    void parse_bms1(const can_frame& frame);
    void parse_bms2(const can_frame& frame);
    void parse_bms3(const can_frame& frame);
    void parse_bms4(const can_frame& frame);
    void parse_bms5(const can_frame& frame);
    void parse_bms6(const can_frame& frame);
    void parse_bms7(const can_frame& frame);
    void parse_bms8(const can_frame& frame);
    void parse_bms9(const can_frame& frame);
    void parse_bms10(const can_frame& frame);
    void parse_bms11(const can_frame& frame);
    void parse_bms21(const can_frame& frame);
    void parse_bms22(const can_frame& frame);

    static uint8_t  u8(const can_frame& f, int off) { return f.data[off]; }
    static uint16_t u16_le(const can_frame& f, int off) { return f.data[off] | (uint16_t(f.data[off+1]) << 8); }

    std::string can_iface_;
    std::shared_ptr<SocketCAN> can_;
    mutable std::shared_mutex mutex_;
    bms::BatteryStatus cached_{};
    std::atomic<bool> connected_{false};
};

} // namespace bms
