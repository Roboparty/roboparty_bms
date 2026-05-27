// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

/**
 * @file bms_driver.hpp
 * @brief Abstract BMS driver interface and battery status data structures.
 * @details Defines the BmsDriver base class with pure virtual accessors for
 *          battery metrics, and the bms::BatteryStatus packed struct shared
 *          across all driver backends.
 */
 
#pragma once

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace bms {

#pragma pack(push, 1)
struct BatteryStatus {
    double voltage;
    double current;
    double temperature;
    double percentage;
    double charge;
    double capacity;
    double design_capacity;
    uint32_t protect_status;
    uint16_t work_state;
    double max_cell_voltage;
    double min_cell_voltage;
    char serial_number[33];
    uint16_t sw_version;
    uint16_t hw_version;
    uint16_t soh;
    uint32_t cycles;
};
#pragma pack(pop)

} // namespace bms

class BmsDriver {
   public:
    BmsDriver() {
        std::vector<spdlog::sink_ptr> sinks;
        sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_st>());
        logger_ = spdlog::get("bms");
        if (!logger_) {
            logger_ = std::make_shared<spdlog::logger>("bms", std::begin(sinks), std::end(sinks));
            spdlog::register_logger(logger_);
        }
    }
    virtual ~BmsDriver() = default;

    static std::shared_ptr<BmsDriver> create_bms(const std::string& bms_type,
                                                  const std::string& socket_path = "/tmp/bms.sock");

    virtual double get_voltage() const = 0;
    virtual double get_current() const = 0;
    virtual double get_temperature() const = 0;
    virtual double get_percentage() const = 0;
    virtual double get_charge() const = 0;
    virtual double get_capacity() const = 0;
    virtual double get_design_capacity() const = 0;
    virtual uint32_t get_protect_status() const = 0;
    virtual uint16_t get_work_state() const = 0;
    virtual double get_max_cell_voltage() const = 0;
    virtual double get_min_cell_voltage() const = 0;
    virtual uint16_t get_soh() const = 0;
    virtual uint32_t get_cycles() const = 0;
    virtual bool is_connected() const = 0;

   protected:
    std::shared_ptr<spdlog::logger> logger_;
};

namespace tws_bms {

class BmsProtocol {
public:
    BmsProtocol(const std::string& port_name, int baud_rate, int timeout_ms = 300);
    ~BmsProtocol();

    bool open();
    void close_port();
    bool is_open() const;

    bool read_basic_info(bms::BatteryStatus& status);
    bool read_version_info(bms::BatteryStatus& status);
    bool read_capacity_info(bms::BatteryStatus& status);
    bool read_serial_number(std::string& sn);
    bool set_discharge_output(bool enable);

private:
    int serial_fd_;
    std::string port_name_;
    int baud_rate_;
    int timeout_ms_;

    uint16_t calculate_crc(const uint8_t *data, size_t len);
    void send_read_request(uint16_t start_addr, uint16_t num_regs);
    bool read_response(std::vector<uint8_t>& buffer, int expected_bytes);

    uint16_t get_u16(const uint8_t* buf, int offset);
    uint32_t get_u32(const uint8_t* buf, int offset);
    int32_t get_i32(const uint8_t* buf, int offset);
    uint16_t get_u16(const std::vector<uint8_t>& buf, int offset);
    uint32_t get_u32(const std::vector<uint8_t>& buf, int offset);
    int32_t get_i32(const std::vector<uint8_t>& buf, int offset);
};

} // namespace tws_bms
