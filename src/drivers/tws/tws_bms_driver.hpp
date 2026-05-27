// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

/**
 * @file tws_bms_driver.hpp
 * @brief TWS BMS driver class declaration.
 * @details Defines the TwsBmsDriver class that extends BmsDriver, adding
 *          Unix socket connectivity and a background thread for receiving
 *          battery status data from the BMS daemon.
 */
 
#pragma once

#include <atomic>
#include <shared_mutex>
#include <string>
#include <thread>

#include "bms_driver.hpp"

class TwsBmsDriver : public BmsDriver {
   public:
    explicit TwsBmsDriver(const std::string& socket_path);
    ~TwsBmsDriver() override;

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
    void reader_loop();
    bool try_connect();
    void disconnect();

    std::string socket_path_;
    int sock_fd_ = -1;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread reader_thread_;
    mutable std::shared_mutex data_mutex_;
    bms::BatteryStatus cached_{};
};
