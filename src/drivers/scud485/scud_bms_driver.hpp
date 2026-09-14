// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

#pragma once

#include <atomic>
#include <shared_mutex>
#include <string>
#include <thread>

#include "bms_driver.hpp"

class ScudBmsDriver : public BmsDriver {
public:
    explicit ScudBmsDriver(const std::string& socket_path);
    ~ScudBmsDriver() override;

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
    uint32_t get_io_state() const override;
    bool is_power_on() const override;
    bool is_connected() const override;
    double get_data_age_ms() const override;

private:
    void reader_loop();
    bool try_connect();
    void disconnect();
    // True while the last record from the daemon is still current
    bool is_data_current() const;

    std::string socket_path_;
    int sock_fd_ = -1;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread reader_thread_;
    mutable std::shared_mutex data_mutex_;
    bms::BatteryStatus cached_{};
    // CLOCK_MONOTONIC microseconds of the last complete record received from
    // the daemon; -1 until the first record of the current session arrives
    // (disconnect resets it, so a fresh socket must earn freshness again)
    std::atomic<int64_t> last_rx_us_{-1};
    bool stale_logged_ = false;  // reader thread only
};
