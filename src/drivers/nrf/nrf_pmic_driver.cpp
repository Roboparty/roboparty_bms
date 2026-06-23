// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "nrf_pmic_driver.hpp"

#include <cstring>
#include <iostream>
#include <sstream>
#include <unordered_map>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

NrfPmicDriver::NrfPmicDriver(const std::string& serial_port, int baud_rate)
    : BmsDriver(), port_(serial_port), baud_rate_(baud_rate) {

    std::memset(&cached_, 0, sizeof(cached_));
    std::memset(&status_, 0, sizeof(status_));

    if (!open_serial()) {
        logger_->warn("nRF PMIC: failed to open {}", port_);
        return;
    }

    logger_->info("nRF PMIC driver started, port={}, baud={}", port_, baud_rate_);

    refresh_status();
    refresh_bms();
    refresh_version();

    running_ = true;
    poll_thread_ = std::thread(&NrfPmicDriver::poll_loop, this);
}

NrfPmicDriver::~NrfPmicDriver() {
    running_ = false;
    if (poll_thread_.joinable()) poll_thread_.join();
    close_serial();
}

bool NrfPmicDriver::open_serial() {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty {};
    tcgetattr(fd_, &tty);

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(fd_, TCSANOW, &tty);
    return true;
}

void NrfPmicDriver::close_serial() {
    if (fd_ >= 0) { close(fd_); fd_ = -1; }
}

bool NrfPmicDriver::send_cmd(const std::string& cmd) {
    if (fd_ < 0) return false;
    std::string full = cmd + "\r\n";
    ssize_t n = write(fd_, full.c_str(), full.size());
    return n == static_cast<ssize_t>(full.size());
}

bool NrfPmicDriver::read_line(std::string& line, int timeout_ms) {
    if (fd_ < 0) return false;

    char buf[256];
    line.clear();

    auto start = std::chrono::steady_clock::now();

    while (true) {
        struct pollfd pfd {fd_, POLLIN, 0};
        int remaining = timeout_ms;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        remaining -= static_cast<int>(elapsed);
        if (remaining <= 0) return false;

        int ret = poll(&pfd, 1, remaining);
        if (ret < 0) return false;
        if (ret == 0) continue;

        ssize_t n = read(fd_, buf, sizeof(buf) - 1);
        if (n <= 0) return false;

        for (ssize_t i = 0; i < n; ++i) {
            line += buf[i];
            if (line.size() >= 2 && line[line.size() - 2] == '\r' && line[line.size() - 1] == '\n') {
                line.resize(line.size() - 2);
                return true;
            }
            if (line.size() >= 1 && line.back() == '\n') {
                line.pop_back();
                if (!line.empty() && line.back() == '\r') line.pop_back();
                return true;
            }
        }
    }
}

bool NrfPmicDriver::send_and_read(const std::string& cmd, std::string& response, int timeout_ms) {
    if (!send_cmd(cmd)) return false;

    bool got_something = false;
    std::string acc;

    while (read_line(acc, timeout_ms)) {
        if (!acc.empty()) {
            response = acc;
            parse_response(acc);
            got_something = true;
        }
    }

    return got_something;
}

void NrfPmicDriver::parse_response(const std::string& line) {
    if (line.rfind("+ESTOP:", 0) == 0) {
        status_.estop.active = (line.find("ACTIVE") != std::string::npos);
    }
    else if (line.rfind("+BMS:", 0) == 0 || line.rfind("+BMS: ", 0) == 0) {
        auto f = [&](const char* key, double& val) {
            auto pos = line.find(key);
            if (pos != std::string::npos) {
                val = std::stod(line.substr(pos + strlen(key)));
            }
        };
        f("VOLT=", status_.bms.voltage);
        f("CURR=", status_.bms.current);
        f("SOC=",  status_.bms.soc);
    }
    else if (line.rfind("+STATUS: VARIANT=", 0) == 0) {
        auto p = line.find("VARIANT=");
        if (p != std::string::npos) {
            p += 8;
            auto comma = line.find(',', p);
        status_.variant.name = line.substr(p, comma == std::string::npos ? line.size() - p : comma - p);
        }
        status_.power48 = line.find("48V=ON")  != std::string::npos;
        status_.power5  = line.find("5V=ON")   != std::string::npos;
        status_.power19 = line.find("19V=ON")  != std::string::npos;
        status_.power12 = line.find("12V=ON")  != std::string::npos;

        status_.variant.has_48v = line.find("48V=") != std::string::npos;
        status_.variant.has_5v  = line.find("5V=")  != std::string::npos;
        status_.variant.has_19v = line.find("19V=") != std::string::npos;
        status_.variant.has_12v = line.find("12V=") != std::string::npos;
    }
    else if (line.rfind("+STATUS: BMS_VOLT=", 0) == 0) {
        auto f = [&](const char* key, double& val) {
            auto pos = line.find(key);
            if (pos != std::string::npos) {
                pos += strlen(key);
                auto end = line.find_first_of(",;", pos);
                if (end == std::string::npos) end = line.size();
                auto num = line.substr(pos, end - pos);
                auto unit = num.find_first_of("VA%");
                if (unit != std::string::npos) num.resize(unit);
                val = std::stod(num);
            }
        };
        f("BMS_VOLT=", status_.bms.voltage);
        f("BMS_CURR=", status_.bms.current);
        f("BMS_SOC=",  status_.bms.soc);
    }
    else if (line.rfind("+ADC:", 0) == 0) {
        for (int i = 0; i < AdcChannels::COUNT; ++i) {
            auto key = std::to_string(i) + "=";
            auto pos = line.find(key);
            if (pos != std::string::npos) {
                pos += key.size();
                auto end = line.find_first_of(",mV", pos);
                if (end == std::string::npos) end = line.size();
                status_.adc.mV[i] = static_cast<uint16_t>(std::stoi(line.substr(pos, end - pos)));
            }
        }
    }
    else if (line.rfind("+WDG:", 0) == 0) {
        status_.wdg_enabled = (line.find("ON") != std::string::npos);
    }
    else if (line.rfind("+VERSION:", 0) == 0) {
        auto p = line.find(':');
        if (p != std::string::npos) {
            status_.fw_version = line.substr(p + 1);
            while (!status_.fw_version.empty() && status_.fw_version[0] == ' ')
                status_.fw_version.erase(0, 1);
        }
    }
}

void NrfPmicDriver::poll_loop() {
    pthread_setname_np(pthread_self(), "nrf_poll");

    while (running_) {
        std::string resp;

        send_and_read("AT+GETBMS?", resp, 300);
        if (!resp.empty()) {
            std::unique_lock lock(mutex_);
            cached_.voltage    = status_.bms.voltage;
            cached_.current    = status_.bms.current;
            cached_.percentage = status_.bms.soc;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        send_and_read("AT+STATUS?", resp, 200);

        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
}

// --- BmsDriver interface ---
double NrfPmicDriver::get_voltage() const {
    std::shared_lock l(mutex_);
    return cached_.voltage;
}
double NrfPmicDriver::get_current() const {
    std::shared_lock l(mutex_);
    return cached_.current;
}
double NrfPmicDriver::get_temperature() const {
    return 0.0; // nRF PMIC doesn't read temperature directly
}
double NrfPmicDriver::get_percentage() const {
    std::shared_lock l(mutex_);
    return cached_.percentage;
}
double NrfPmicDriver::get_charge() const        { return 0.0; }
double NrfPmicDriver::get_capacity() const      { return 0.0; }
double NrfPmicDriver::get_design_capacity() const { return 0.0; }
uint32_t NrfPmicDriver::get_protect_status() const { return 0; }
uint16_t NrfPmicDriver::get_work_state() const     { return 0; }
double NrfPmicDriver::get_max_cell_voltage() const { return 0.0; }
double NrfPmicDriver::get_min_cell_voltage() const { return 0.0; }
uint16_t NrfPmicDriver::get_soh() const            { return 0; }
uint32_t NrfPmicDriver::get_cycles() const         { return 0; }
bool NrfPmicDriver::is_connected() const           { return fd_ >= 0; }

// --- Power control ---
bool NrfPmicDriver::set_power48(bool on) {
    std::string resp;
    return send_and_read(on ? "AT+POWER48=1" : "AT+POWER48=0", resp);
}
bool NrfPmicDriver::set_power5(bool on) {
    std::string resp;
    return send_and_read(on ? "AT+POWER5=1" : "AT+POWER5=0", resp);
}
bool NrfPmicDriver::set_power19(bool on) {
    std::string resp;
    return send_and_read(on ? "AT+POWER19=1" : "AT+POWER19=0", resp);
}
bool NrfPmicDriver::set_power12(bool on) {
    std::string resp;
    return send_and_read(on ? "AT+POWER12=1" : "AT+POWER12=0", resp);
}

bool NrfPmicDriver::get_power48() const { std::shared_lock l(mutex_); return status_.power48; }
bool NrfPmicDriver::get_power5()  const { std::shared_lock l(mutex_); return status_.power5;  }
bool NrfPmicDriver::get_power19() const { std::shared_lock l(mutex_); return status_.power19; }
bool NrfPmicDriver::get_power12() const { std::shared_lock l(mutex_); return status_.power12; }

bool NrfPmicDriver::estop() {
    std::string resp;
    return send_and_read("AT+ESTOP=1", resp);
}
bool NrfPmicDriver::clear_estop() {
    std::string resp;
    return send_and_read("AT+ESTOP=0", resp);
}
bool NrfPmicDriver::query_estop() {
    std::string resp;
    return send_and_read("AT+ESTOP?", resp);
}

bool NrfPmicDriver::set_watchdog(bool on) {
    std::string resp;
    return send_and_read(on ? "AT+WDG=1" : "AT+WDG=0", resp);
}
bool NrfPmicDriver::query_watchdog() {
    std::string resp;
    return send_and_read("AT+WDG?", resp);
}

bool NrfPmicDriver::refresh_status() {
    std::string resp;
    return send_and_read("AT+STATUS?", resp);
}
bool NrfPmicDriver::refresh_bms() {
    std::string resp;
    return send_and_read("AT+GETBMS?", resp);
}
bool NrfPmicDriver::refresh_adc(int channel) {
    std::string resp;
    if (channel < 0) {
        return send_and_read("AT+ADC?", resp);
    } else {
        return send_and_read("AT+ADC=" + std::to_string(channel), resp);
    }
}
bool NrfPmicDriver::refresh_version() {
    std::string resp;
    return send_and_read("AT+VERSION?", resp);
}
