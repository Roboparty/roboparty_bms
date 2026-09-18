// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace bms {

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baud_rate, int vmin, int vtime) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty {};
    if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

    speed_t speed;
    switch (baud_rate) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break; // required by the SCUD485 protocol
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default:     speed = B9600;   break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= CS8;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = static_cast<cc_t>(vmin);
    tty.c_cc[VTIME] = static_cast<cc_t>(vtime);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }
    fcntl(fd_, F_SETFL, FNDELAY);

    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    // forward_fd_ is intentionally left alone: the daemon reopens the bus
    // after repeated read failures and the tap must survive the cycle.
}

ssize_t SerialPort::write_raw(const uint8_t* data, size_t len) {
    if (fd_ < 0) return -1;
    return ::write(fd_, data, len);
}

ssize_t SerialPort::write_raw(const std::vector<uint8_t>& data) {
    return write_raw(data.data(), data.size());
}

void SerialPort::set_forward_fd(int fd) {
    std::lock_guard<std::mutex> lock(forward_mutex_);
    // Idempotent: the daemon re-asserts the tap once per loop round, and
    // clearing the queue on every call would throw away bytes that a stalled
    // destination has not taken yet.
    if (forward_fd_ == fd) return;
    forward_fd_ = fd;
    forward_pending_.clear();
    forward_drop_logged_ = false;
}

void SerialPort::tee(const uint8_t* data, size_t len) {
    if (len == 0) return;

    std::lock_guard<std::mutex> lock(forward_mutex_);
    if (forward_fd_ < 0) return;

    if (forward_pending_.size() + len > kForwardMaxPending) {
        // The destination is not draining. Drop what is already queued and
        // carry on: forward data is a monitor feed, the BMS read loop that
        // called us is not allowed to wait for it.
        forward_pending_.clear();
        if (!forward_drop_logged_) {
            std::cerr << "[SerialPort] forward destination stalled; dropping data"
                      << std::endl;
            forward_drop_logged_ = true;
        }
        if (len > kForwardMaxPending) return; // single read larger than the cap
    }

    forward_pending_.insert(forward_pending_.end(), data, data + len);
    drain_forward_locked();
}

void SerialPort::drain_forward_locked() {
    while (!forward_pending_.empty()) {
        ssize_t n = ::write(forward_fd_, forward_pending_.data(),
                            forward_pending_.size());
        if (n > 0) {
            forward_pending_.erase(forward_pending_.begin(),
                                   forward_pending_.begin() + n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;

        // Destination error (EIO/EPIPE/EBADF/...). Drop what is queued and
        // complain once, but keep the descriptor: detaching here would leave
        // the fd set to -1 while the daemon still believes the tap is armed,
        // so a transient error would silently end forwarding for good.
        if (!forward_drop_logged_) {
            std::cerr << "[SerialPort] forward destination error (errno "
                      << errno << "); dropping data" << std::endl;
            forward_drop_logged_ = true;
        }
        forward_pending_.clear();
        return;
    }
    forward_drop_logged_ = false;
}

ssize_t SerialPort::read_raw(uint8_t* buf, size_t max_len, int timeout_ms) {
    if (fd_ < 0) return -1;

    struct pollfd pfd {fd_, POLLIN, 0};
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return 0;

    ssize_t n = ::read(fd_, buf, max_len);
    if (n > 0) tee(buf, static_cast<size_t>(n));
    return n;
}

ssize_t SerialPort::read_teed(uint8_t* buf, size_t max_len) {
    if (fd_ < 0) return -1;

    ssize_t n = ::read(fd_, buf, max_len);
    // tee() runs only on success, so a failed read still reports the errno
    // the caller's EAGAIN handling depends on.
    if (n > 0) tee(buf, static_cast<size_t>(n));
    return n;
}

void SerialPort::flush() {
    if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
}

} // namespace bms
