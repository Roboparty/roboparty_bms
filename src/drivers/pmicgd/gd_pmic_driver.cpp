// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "gd_pmic_driver.hpp"

GdPmicDriver::GdPmicDriver(const std::string& serial_port, int baud_rate) {
    (void)serial_port;
    (void)baud_rate;
}

GdPmicDriver::~GdPmicDriver() = default;

double GdPmicDriver::get_voltage() const         { return 0.0; }
double GdPmicDriver::get_current() const         { return 0.0; }
double GdPmicDriver::get_temperature() const     { return 0.0; }
double GdPmicDriver::get_percentage() const      { return 0.0; }
double GdPmicDriver::get_charge() const          { return 0.0; }
double GdPmicDriver::get_capacity() const        { return 0.0; }
double GdPmicDriver::get_design_capacity() const { return 0.0; }
uint32_t GdPmicDriver::get_protect_status() const { return 0; }
uint16_t GdPmicDriver::get_work_state() const    { return 0; }
double GdPmicDriver::get_max_cell_voltage() const { return 0.0; }
double GdPmicDriver::get_min_cell_voltage() const { return 0.0; }
uint16_t GdPmicDriver::get_soh() const           { return 0; }
uint32_t GdPmicDriver::get_cycles() const        { return 0; }
bool GdPmicDriver::is_connected() const          { return false; }
double GdPmicDriver::get_data_age_ms() const     { return -1.0; }

bool GdPmicDriver::set_power48(bool on) { (void)on; return false; }
bool GdPmicDriver::set_power5(bool on)  { (void)on; return false; }
bool GdPmicDriver::set_power19(bool on) { (void)on; return false; }
bool GdPmicDriver::set_power12(bool on) { (void)on; return false; }

bool GdPmicDriver::get_power48() const { return false; }
bool GdPmicDriver::get_power5()  const { return false; }
bool GdPmicDriver::get_power19() const { return false; }
bool GdPmicDriver::get_power12() const { return false; }

bool GdPmicDriver::estop()       { return false; }
bool GdPmicDriver::clear_estop() { return false; }
bool GdPmicDriver::query_estop() { return false; }

bool GdPmicDriver::set_watchdog(bool on) { (void)on; return false; }
bool GdPmicDriver::query_watchdog()      { return false; }

bool GdPmicDriver::refresh_status()          { return false; }
bool GdPmicDriver::refresh_bms()             { return false; }
bool GdPmicDriver::refresh_adc(int channel)  { (void)channel; return false; }
bool GdPmicDriver::refresh_version()         { return false; }
