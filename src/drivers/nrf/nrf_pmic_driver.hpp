// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "bms_driver.hpp"

struct EscStatus {
    bool active = false;
};

struct BmsCache {
    double voltage = 0;
    double current = 0;
    double soc = 0;
};

struct VariantInfo {
    std::string name;
    bool has_48v = false;
    bool has_5v  = false;
    bool has_19v = false;
    bool has_12v = false;
    bool has_ws2812 = false;
};

struct AdcChannels {
    static constexpr int COUNT = 6;
    uint16_t mV[COUNT] = {};
};

struct SystemStatus {
    EscStatus estop;
    VariantInfo variant;
    BmsCache bms;
    AdcChannels adc;
    bool power48 = false;
    bool power5  = false;
    bool power19 = false;
    bool power12 = false;
    bool wdg_enabled = false;
    std::string fw_version;
};

class NrfPmicDriver : public BmsDriver {
public:
    explicit NrfPmicDriver(const std::string& serial_port, int baud_rate = 115200);
    ~NrfPmicDriver() override;

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

    bool set_power48(bool on);
    bool set_power5(bool on);
    bool set_power19(bool on);
    bool set_power12(bool on);

    bool get_power48() const;
    bool get_power5()  const;
    bool get_power19() const;
    bool get_power12() const;

    bool estop();
    bool clear_estop();
    bool query_estop();

    bool set_watchdog(bool on);
    bool query_watchdog();

    bool refresh_status();
    bool refresh_bms();
    bool refresh_adc(int channel = -1);
    bool refresh_version();

    const SystemStatus& status() const { return status_; }

private:
    bool open_serial();
    void close_serial();
    bool send_cmd(const std::string& cmd);
    bool read_line(std::string& line, int timeout_ms);
    bool send_and_read(const std::string& cmd, std::string& response, int timeout_ms = 500);
    void parse_response(const std::string& line);
    void poll_loop();

    std::string port_;
    int baud_rate_;
    int fd_ = -1;

    std::atomic<bool> running_{false};
    std::thread poll_thread_;
    mutable std::shared_mutex mutex_;

    SystemStatus status_;
    bms::BatteryStatus cached_{};
};
