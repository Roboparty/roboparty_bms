// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "bms_driver.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <poll.h>

namespace gf_bms {

#define FUNC_READ 0x03
#define FUNC_WRITE_MULTI 0x10

static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

GfBmsProtocol::GfBmsProtocol(const std::string& port_name, int baud_rate,
                             int timeout_ms, uint8_t dev_addr)
    : serial_fd_(-1), port_name_(port_name), baud_rate_(baud_rate),
      dev_addr_(dev_addr), timeout_ms_(timeout_ms) {}

GfBmsProtocol::~GfBmsProtocol() { close_port(); }

bool GfBmsProtocol::open() {
    serial_fd_ = ::open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ < 0) return false;

    struct termios options;
    if (tcgetattr(serial_fd_, &options) != 0) {
        close_port();
        return false;
    }

    speed_t baud = (baud_rate_ == 9600) ? B9600 : B115200;
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
        close_port();
        return false;
    }

    fcntl(serial_fd_, F_SETFL, FNDELAY);
    return true;
}

void GfBmsProtocol::close_port() {
    if (serial_fd_ >= 0) { ::close(serial_fd_); serial_fd_ = -1; }
}

bool GfBmsProtocol::is_open() const { return serial_fd_ >= 0; }

void GfBmsProtocol::flush() {
    if (serial_fd_ >= 0) tcflush(serial_fd_, TCIOFLUSH);
}

uint16_t GfBmsProtocol::calculate_crc(const uint8_t* data, size_t len) {
    uint8_t ucCRCHi = 0xFF, ucCRCLo = 0xFF;
    int iIndex;
    while (len--) {
        iIndex = ucCRCLo ^ *(data++);
        ucCRCLo = static_cast<uint8_t>(ucCRCHi ^ aucCRCHi[iIndex]);
        ucCRCHi = aucCRCLo[iIndex];
    }
    return static_cast<uint16_t>(ucCRCHi << 8) | ucCRCLo;
}

void GfBmsProtocol::send_read_request(uint16_t start_addr, uint16_t num_regs) {
    uint8_t frame[8] = {dev_addr_, FUNC_READ,
                        static_cast<uint8_t>(start_addr >> 8),
                        static_cast<uint8_t>(start_addr),
                        static_cast<uint8_t>(num_regs >> 8),
                        static_cast<uint8_t>(num_regs)};
    uint16_t crc = calculate_crc(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;
    ssize_t _ = write(serial_fd_, frame, 8);
    (void)_;
}

bool GfBmsProtocol::read_response(std::vector<uint8_t>& buffer,
                                   int expected_bytes) {
    if (serial_fd_ < 0) return false;

    buffer.assign(expected_bytes, 0);
    int total_read = 0;
    struct pollfd pfd = {serial_fd_, POLLIN, 0};

    while (total_read < expected_bytes) {
        int ret = poll(&pfd, 1, timeout_ms_);
        if (ret > 0) {
            int n = read(serial_fd_, buffer.data() + total_read,
                         expected_bytes - total_read);
            if (n > 0) {
                total_read += n;
            } else if (n < 0 && errno != EAGAIN) {
                break;
            } else {
                break;
            }
        } else {
            break;
        }
    }

    if (total_read < 5) return false;

    uint16_t received_crc = buffer[total_read - 2] |
                            (buffer[total_read - 1] << 8);
    uint16_t calc_crc = calculate_crc(buffer.data(), total_read - 2);

    if (received_crc != calc_crc) return false;
    if (buffer[1] & 0x80) return false;
    return total_read == expected_bytes;
}

uint16_t GfBmsProtocol::get_u16_be(const uint8_t* buf, int offset) {
    return static_cast<uint16_t>(buf[offset] << 8) | buf[offset + 1];
}

int16_t GfBmsProtocol::get_i16_be(const uint8_t* buf, int offset) {
    return static_cast<int16_t>(get_u16_be(buf, offset));
}

uint32_t GfBmsProtocol::get_u32_be(const uint8_t* buf, int offset) {
    uint16_t lo = get_u16_be(buf, offset);
    uint16_t hi = get_u16_be(buf, offset + 2);
    return lo | (static_cast<uint32_t>(hi) << 16);
}

uint16_t GfBmsProtocol::get_u16_be(const std::vector<uint8_t>& buf,
                                    int offset) {
    return get_u16_be(buf.data(), offset);
}

int16_t GfBmsProtocol::get_i16_be(const std::vector<uint8_t>& buf,
                                   int offset) {
    return get_i16_be(buf.data(), offset);
}

uint32_t GfBmsProtocol::get_u32_be(const std::vector<uint8_t>& buf,
                                    int offset) {
    return get_u32_be(buf.data(), offset);
}

bool GfBmsProtocol::write_multiple_registers(
    uint16_t start_addr, const std::vector<uint16_t>& values) {
    flush();
    usleep(10000);

    uint16_t count = static_cast<uint16_t>(values.size());
    size_t data_bytes = count * 2;
    size_t frame_size = 9 + data_bytes;
    std::vector<uint8_t> frame(frame_size);

    frame[0] = dev_addr_;
    frame[1] = FUNC_WRITE_MULTI;
    frame[2] = static_cast<uint8_t>(start_addr >> 8);
    frame[3] = static_cast<uint8_t>(start_addr);
    frame[4] = static_cast<uint8_t>(count >> 8);
    frame[5] = static_cast<uint8_t>(count);
    frame[6] = static_cast<uint8_t>(data_bytes);

    for (size_t i = 0; i < count; i++) {
        frame[7 + i * 2] = static_cast<uint8_t>(values[i] >> 8);
        frame[8 + i * 2] = static_cast<uint8_t>(values[i]);
    }

    uint16_t crc = calculate_crc(frame.data(), frame_size - 2);
    frame[frame_size - 2] = crc & 0xFF;
    frame[frame_size - 1] = crc >> 8;

    ssize_t _ = write(serial_fd_, frame.data(), frame_size);
    (void)_;

    std::vector<uint8_t> resp;
    return read_response(resp, 8);
}

bool GfBmsProtocol::read_basic_info(bms::BatteryStatus& status) {
    flush();
    usleep(10000);

    send_read_request(114, 20);
    std::vector<uint8_t> buf;
    if (!read_response(buf, 45)) return false;

    status.voltage = get_u16_be(buf, 3) / 100.0;
    status.current = get_i16_be(buf, 5) / 100.0;
    status.temperature = get_u16_be(buf, 41) - 273.15;
    status.protect_status = get_u32_be(buf, 21);
    status.work_state = get_u16_be(buf, 11);
    status.max_cell_voltage = get_u16_be(buf, 35) / 1000.0;
    status.min_cell_voltage = get_u16_be(buf, 37) / 1000.0;
    return true;
}

bool GfBmsProtocol::read_version_info(bms::BatteryStatus& status) {
    flush();
    usleep(10000);

    send_read_request(1, 2);
    std::vector<uint8_t> buf;
    if (!read_response(buf, 9)) return false;

    status.sw_version = get_u16_be(buf, 3);
    status.hw_version = get_u16_be(buf, 5);

    usleep(50000);
    flush();
    send_read_request(117, 1);
    if (read_response(buf, 7)) status.soh = get_u16_be(buf, 3);

    usleep(50000);
    flush();
    send_read_request(129, 1);
    if (read_response(buf, 7)) status.cycles = get_u16_be(buf, 3);

    return true;
}

bool GfBmsProtocol::read_capacity_info(bms::BatteryStatus& status) {
    flush();
    usleep(10000);

    send_read_request(116, 14);
    std::vector<uint8_t> buf;
    if (!read_response(buf, 33)) return false;

    status.percentage = get_u16_be(buf, 3) / 100.0;
    status.soh = get_u16_be(buf, 5);
    status.charge = get_u16_be(buf, 23) / 100.0;
    status.capacity = get_u16_be(buf, 27) / 100.0;
    return true;
}

bool GfBmsProtocol::read_serial_number(std::string& sn) {
    flush();
    usleep(10000);

    send_read_request(100, 12);
    std::vector<uint8_t> buf;
    if (!read_response(buf, 29)) return false;

    char sn_str[25];
    for (int i = 0; i < 24; i++) sn_str[i] = buf[3 + i];
    sn_str[24] = '\0';
    sn = std::string(sn_str);
    return true;
}

bool GfBmsProtocol::set_discharge_output(bool) { return false; }

} // namespace gf_bms
