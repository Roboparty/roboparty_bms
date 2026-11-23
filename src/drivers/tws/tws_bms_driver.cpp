// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

/**
 * @file tws_bms_driver.cpp
 * @brief TWS BMS driver implementation using Unix domain socket communication.
 * @details Implements BmsDriver for the TWS BMS. Connects to the bms_daemon
 *          process over a Unix socket and caches periodic battery status updates
 *          received in a background reader thread.
 */
 
#include "tws_bms_driver.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>

#include <chrono>
#include <cstring>
#include <thread>

TwsBmsDriver::TwsBmsDriver(const std::string& socket_path)
    : BmsDriver(), socket_path_(socket_path) {
    std::memset(&cached_, 0, sizeof(cached_));
    running_ = true;
    reader_thread_ = std::thread(&TwsBmsDriver::reader_loop, this);
    logger_->info("TWS BMS driver started, socket={}", socket_path_);
}

TwsBmsDriver::~TwsBmsDriver() {
    running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    disconnect();
}

bool TwsBmsDriver::try_connect() {
    if (sock_fd_ >= 0) return true;

    sock_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock_fd_ < 0) return false;

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(sock_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock_fd_);
        sock_fd_ = -1;
        return false;
    }

    connected_ = true;
    logger_->info("Connected to BMS daemon at {}", socket_path_);
    return true;
}

void TwsBmsDriver::disconnect() {
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }
    if (info_sock_fd_ >= 0) {
        close(info_sock_fd_);
        info_sock_fd_ = -1;
    }
    connected_ = false;
}

bool TwsBmsDriver::try_connect_info() {
    if (info_sock_fd_ >= 0) return true;

    std::string path = socket_path_ + ".info";
    info_sock_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (info_sock_fd_ < 0) return false;

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(info_sock_fd_, reinterpret_cast<struct sockaddr*>(&addr),
                sizeof(addr)) < 0) {
        close(info_sock_fd_);
        info_sock_fd_ = -1;
        return false;
    }
    return true;
}

void TwsBmsDriver::reader_loop() {
    pthread_setname_np(pthread_self(), "bms_reader");

    while (running_) {
        if (!try_connect()) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }
        // Best effort: an older daemon may serve no info socket; it is
        // retried every round until it appears.
        try_connect_info();

        struct pollfd pfds[2] = {{sock_fd_, POLLIN, 0},
                                 {info_sock_fd_, POLLIN, 0}};
        int nfds = info_sock_fd_ >= 0 ? 2 : 1;
        int ret = poll(pfds, nfds, 500);
        if (ret <= 0) continue;

        if (pfds[0].revents & POLLIN) {
            bms::BatteryStatus wire;
            ssize_t n = read(sock_fd_, &wire, sizeof(wire));
            if (n == static_cast<ssize_t>(sizeof(wire))) {
                std::unique_lock lock(data_mutex_);
                cached_ = wire;
            } else {
                logger_->warn("BMS daemon disconnected, reconnecting...");
                disconnect();
                continue;
            }
        }
        if (nfds == 2 && (pfds[1].revents & (POLLIN | POLLERR | POLLHUP))) {
            bms::IdentityRecord id;
            ssize_t n = read(info_sock_fd_, &id, sizeof(id));
            if (n == static_cast<ssize_t>(sizeof(id))) {
                std::unique_lock lock(data_mutex_);
                ident_ = id;
            } else {
                // Identity socket dropped; re-attach next round.
                close(info_sock_fd_);
                info_sock_fd_ = -1;
            }
        }
    }
}

double TwsBmsDriver::get_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.voltage;
}

double TwsBmsDriver::get_current() const {
    std::shared_lock lock(data_mutex_);
    return cached_.current;
}

double TwsBmsDriver::get_temperature() const {
    std::shared_lock lock(data_mutex_);
    return cached_.temperature;
}

double TwsBmsDriver::get_percentage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.percentage;
}

double TwsBmsDriver::get_charge() const {
    std::shared_lock lock(data_mutex_);
    return cached_.charge;
}

double TwsBmsDriver::get_capacity() const {
    std::shared_lock lock(data_mutex_);
    return cached_.capacity;
}

double TwsBmsDriver::get_design_capacity() const {
    std::shared_lock lock(data_mutex_);
    return cached_.design_capacity;
}

uint32_t TwsBmsDriver::get_protect_status() const {
    std::shared_lock lock(data_mutex_);
    return cached_.protect_status;
}

uint16_t TwsBmsDriver::get_work_state() const {
    std::shared_lock lock(data_mutex_);
    return cached_.work_state;
}

double TwsBmsDriver::get_max_cell_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.max_cell_voltage;
}

double TwsBmsDriver::get_min_cell_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.min_cell_voltage;
}

uint16_t TwsBmsDriver::get_soh() const {
    std::shared_lock lock(data_mutex_);
    return cached_.soh;
}

uint32_t TwsBmsDriver::get_cycles() const {
    std::shared_lock lock(data_mutex_);
    return cached_.cycles;
}

uint32_t TwsBmsDriver::get_io_state() const {
    std::shared_lock lock(data_mutex_);
    return cached_.io_state;
}

bool TwsBmsDriver::is_power_on() const {
    std::shared_lock lock(data_mutex_);
    return cached_.power_on != 0;
}

bool TwsBmsDriver::is_connected() const {
    return connected_.load();
}

uint16_t TwsBmsDriver::get_sw_version() const {
    std::shared_lock lock(data_mutex_);
    return cached_.sw_version;
}

uint16_t TwsBmsDriver::get_hw_version() const {
    std::shared_lock lock(data_mutex_);
    return cached_.hw_version;
}

std::string TwsBmsDriver::get_serial_number() const {
    std::shared_lock lock(data_mutex_);
    // NUL-terminated: the daemon zeroes the record and uses bounded strncpy.
    return cached_.serial_number;
}

std::string TwsBmsDriver::get_app_version() const {
    std::shared_lock lock(data_mutex_);
    return ident_.app_version;
}
