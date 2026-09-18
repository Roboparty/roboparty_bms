// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#pragma once

#include <cstdint>
#include <mutex>
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

    // Best-effort byte tap: everything this port reads is also written to
    // `fd`. This exists so bms_daemon can forward the BMS bus to a second
    // UART, instead of letting a separate process open the same tty -- two
    // readers on one tty steal bytes from each other, which is exactly the
    // data loss a second, competing reader used to cause on /dev/ttyS3.
    // `fd` must be non-blocking, otherwise a stalled destination blocks the
    // protocol read loop that calls us (a SerialPort opened via open() is).
    // Pass -1 to detach. Deliberately not cleared by open()/close(): the
    // daemon reopens the bus on repeated read failures and the tap has to
    // survive that.
    void set_forward_fd(int fd);
    int forward_fd() const { return forward_fd_; }

    // A plain ::read with the tap above applied, for callers that poll the
    // raw fd themselves (the TWS and GF485 protocol loops). The return value
    // and errno are exactly those of the underlying ::read, so existing
    // `errno != EAGAIN` handling keeps working unchanged.
    ssize_t read_teed(uint8_t* buf, size_t max_len);

    void flush();

private:
    void tee(const uint8_t* data, size_t len);
    void drain_forward_locked();

    int fd_ = -1;

    int forward_fd_ = -1;
    std::vector<uint8_t> forward_pending_;
    std::mutex forward_mutex_;
    bool forward_drop_logged_ = false;

    // The tap is invoked from inside the protocol read loop, so it must never
    // block or spin. Bytes the destination cannot take yet are held here and
    // dropped once past this cap -- losing forward data is acceptable, slowing
    // the BMS read loop down is not.
    static constexpr size_t kForwardMaxPending = 8192;
};

} // namespace bms
