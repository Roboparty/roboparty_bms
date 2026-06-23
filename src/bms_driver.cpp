// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

/**
 * @file bms_driver.cpp
 * @brief Factory implementation for creating BMS driver instances.
 * @details Provides BmsDriver::create_bms() to instantiate the appropriate driver
 *          backend based on the requested BMS type (e.g., TWS).
 */
 
#include "bms_driver.hpp"

#include "gf_bms_driver.hpp"
#include "gfcan_bms_driver.hpp"
#include "nrf_pmic_driver.hpp"
#include "tws_bms_driver.hpp"

std::shared_ptr<BmsDriver> BmsDriver::create_bms(const std::string& bms_type,
                                                  const std::string& socket_path) {
    if (bms_type == "TWS") {
        return std::make_shared<TwsBmsDriver>(socket_path);
    } else if (bms_type == "GF485") {
        return std::make_shared<GfBmsDriver>(
            socket_path.empty() ? "/tmp/gf_bms.sock" : socket_path);
    } else if (bms_type == "GFCAN") {
        return std::make_shared<bms::GfCanBmsDriver>(
            socket_path.empty() ? "can0" : socket_path);
    } else if (bms_type == "NRF") {
        return std::make_shared<NrfPmicDriver>(
            socket_path.empty() ? "/dev/ttyACM0" : socket_path);
    } else {
        throw std::runtime_error("BMS type not supported: " + bms_type);
    }
}
