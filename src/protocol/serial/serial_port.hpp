// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#pragma once

#include <string>
#include <vector>

namespace bms {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    bool open(const std::string& port, int baud_rate, int vmin = 0, int vtime = 0);
    void close();
    bool is_open() const { return fd_ >= 0; }
    int fd() const { return fd_; }

    ssize_t write_raw(const uint8_t* data, size_t len);
    ssize_t write_raw(const std::vector<uint8_t>& data);
    ssize_t read_raw(uint8_t* buf, size_t max_len, int timeout_ms);

    void flush();

private:
    int fd_ = -1;
};

} // namespace bms
