// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "serial_port.hpp"

#include <cstring>
#include <fcntl.h>
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
    case 115200: speed = B115200; break;
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
}

ssize_t SerialPort::write_raw(const uint8_t* data, size_t len) {
    if (fd_ < 0) return -1;
    return ::write(fd_, data, len);
}

ssize_t SerialPort::write_raw(const std::vector<uint8_t>& data) {
    return write_raw(data.data(), data.size());
}

ssize_t SerialPort::read_raw(uint8_t* buf, size_t max_len, int timeout_ms) {
    if (fd_ < 0) return -1;

    struct pollfd pfd {fd_, POLLIN, 0};
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return 0;

    return ::read(fd_, buf, max_len);
}

void SerialPort::flush() {
    if (fd_ >= 0) tcflush(fd_, TCIOFLUSH);
}

} // namespace bms
