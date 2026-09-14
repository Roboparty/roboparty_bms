// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 changchuanyong

/**
 * @file scud_bms_driver.cpp
 * @brief SCUD 485 protocol (daemon side) and Unix socket client driver.
 * @details Frame: 0xFE 0xFE | CMD | LEN | data | CRC_H CRC_L | 0xBB, with
 *          CRC16-CCITT (poly 0x1201, init 0); multi-byte fields big-endian.
 */

#include "scud_bms_driver.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <poll.h>

#include <chrono>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <thread>
#include <vector>

namespace scud_bms {

namespace {
constexpr uint8_t FRAME_HEAD = 0xFE;          // both head bytes are 0xFE
constexpr uint8_t FRAME_TAIL = 0xBB;          // tail marker
constexpr uint8_t QUERY_PARAM = 0x01;         // query parameter is fixed 0x01
constexpr uint8_t CMD_USER_INFO = 0x61;       // user communication info
constexpr uint8_t CMD_BATTERY_SUMMARY = 0x31; // battery summary info
constexpr int LEN_USER_INFO = 28;
constexpr int LEN_BATTERY_SUMMARY = 38;
} // namespace

ScudBmsProtocol::ScudBmsProtocol(const std::string& port_name, int baud_rate,
                                 int timeout_ms)
    : port_name_(port_name), baud_rate_(baud_rate), timeout_ms_(timeout_ms) {}

ScudBmsProtocol::~ScudBmsProtocol() { close_port(); }

bool ScudBmsProtocol::open() {
    return serial_.open(port_name_, baud_rate_);
}

void ScudBmsProtocol::close_port() { serial_.close(); }

bool ScudBmsProtocol::is_open() const { return serial_.is_open(); }

void ScudBmsProtocol::flush() { serial_.flush(); }

// Reference algorithm from the protocol spec section 3.2.2: CRC16-CCITT,
// polynomial 0x1201, init 0. Byte-equivalent to CRC-16/XMODEM.
uint16_t ScudBmsProtocol::crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc = static_cast<uint16_t>((crc >> 8) | (crc << 8));
        crc ^= data[i];
        crc ^= static_cast<uint16_t>((crc & 0xFF) >> 4);
        crc ^= static_cast<uint16_t>((crc << 8) << 4);
        crc ^= static_cast<uint16_t>(((crc & 0xFF) << 4) << 1);
    }
    return crc;
}

// Command sequence: FE FE | CMD | 01 | 01 | CRC_H CRC_L | BB
bool ScudBmsProtocol::send_query(uint8_t cmd) {
    uint8_t body[3] = {cmd, 1, QUERY_PARAM};
    uint16_t crc = crc16_ccitt(body, sizeof(body));

    uint8_t frame[8] = {
        FRAME_HEAD, FRAME_HEAD, body[0], body[1], body[2],
        static_cast<uint8_t>(crc >> 8), static_cast<uint8_t>(crc & 0xFF),
        FRAME_TAIL};

    flush();
    usleep(10000);
    return serial_.write_raw(frame, sizeof(frame)) ==
           static_cast<ssize_t>(sizeof(frame));
}

// Read and validate one response frame; data receives the payload only
// (no head, CRC or tail bytes).
//
// The whole query shares a single timeout_ms_ deadline and advances by
// "scanning until the target frame shows up":
//   - half-duplex 485 converters commonly echo our own command frame back
//     into RX (same 0x61/0x31 command byte, but LEN=1)
//   - other slaves on the bus, and a late reply to the previous query, also
//     reach the receive buffer before the reply we are waiting for
// The old implementation failed the whole query on the first CMD/LEN mismatch
// and would never receive anything on such a bus. Head sync now requires
// FE FE + the expected command byte; non-target frames are dropped in place
// and the byte-wise scan continues until the target frame arrives or the
// deadline expires. Advancing one byte at a time keeps frame boundaries
// unambiguous, and the EAGAIN race between poll() and read() counts as
// "nothing readable this round" rather than an error.
bool ScudBmsProtocol::read_frame(std::vector<uint8_t>& data, uint8_t expect_cmd,
                                 int expect_len) {
    if (!serial_.is_open()) return false;

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
    auto remaining_ms = [&deadline]() -> int {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) return 0;
        return static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
                .count());
    };

    enum Stage : int { kHead1, kHead2, kCmd, kLen, kPayload, kCrcHi, kCrcLo,
                       kTail };
    Stage stage = kHead1;
    uint8_t cmd = 0, len = 0, crc_hi = 0, crc_lo = 0;
    std::vector<uint8_t> body;

    for (;;) {
        const int wait = remaining_ms();
        if (wait <= 0) return false; // no target frame before the deadline

        uint8_t byte = 0;
        const ssize_t n = serial_.read_raw(&byte, 1, wait);
        if (n != 1) {
            if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EINTR))) {
                continue; // nothing readable: keep waiting, timeout at loop top
            }
            return false; // link error (unplug -> EIO); let the daemon reopen
        }

        switch (stage) {
        case kHead1:
            if (byte == FRAME_HEAD) stage = kHead2;
            break;
        case kHead2:
            if (byte == FRAME_HEAD) {
                stage = kCmd;
                body.clear();
            } else {
                stage = kHead1; // stray byte after a single 0xFE: resync
            }
            break;
        case kCmd:
            // Head sync is a strict prefix match on "FE FE <expect_cmd>". A
            // non-target 0xFE can still be the second head byte of a later
            // frame, so stay in kCmd instead of falling back to kHead2: that
            // would miss a real frame preceded by an odd number of 0xFE bytes.
            if (byte == expect_cmd) {
                cmd = byte;
                stage = kLen;
            } else if (byte == FRAME_HEAD) {
                stage = kCmd;
            } else {
                stage = kHead1;
            }
            break;
        case kLen:
            len = byte;
            // Drop the frame in place when LEN is not what we expect, and
            // never consume the number of bytes it claims: a noise-corrupted
            // LEN, or a foreign frame advertising a large one, would swallow
            // the real frame that follows it.
            if (len != static_cast<uint8_t>(expect_len)) {
                stage = (len == FRAME_HEAD) ? kHead2 : kHead1;
                break;
            }
            body.clear();
            stage = (len == 0) ? kCrcHi : kPayload;
            break;
        case kPayload:
            body.push_back(byte);
            if (static_cast<int>(body.size()) == len) stage = kCrcHi;
            break;
        case kCrcHi:
            crc_hi = byte;
            stage = kCrcLo;
            break;
        case kCrcLo:
            crc_lo = byte;
            stage = kTail;
            break;
        case kTail: {
            // cmd/LEN already match by construction at this point; what is
            // left to check is the tail byte and the CRC.
            const bool shape_ok = (cmd == expect_cmd) &&
                                  (static_cast<int>(len) == expect_len) &&
                                  (byte == FRAME_TAIL);
            if (shape_ok) {
                // CRC covers CMD + LEN + payload (head and tail excluded)
                std::vector<uint8_t> crcbuf;
                crcbuf.reserve(2 + body.size());
                crcbuf.push_back(cmd);
                crcbuf.push_back(len);
                crcbuf.insert(crcbuf.end(), body.begin(), body.end());
                const uint16_t calc = crc16_ccitt(crcbuf.data(), crcbuf.size());
                const uint16_t recv =
                    static_cast<uint16_t>((crc_hi << 8) | crc_lo);
                if (calc == recv) {
                    data.assign(body.begin(), body.end());
                    return true;
                }
            }
            stage = (byte == FRAME_HEAD) ? kHead2 : kHead1;
            break;
        }
        }
    }
}

bool ScudBmsProtocol::query_0x61(std::vector<uint8_t>& data) {
    if (!send_query(CMD_USER_INFO)) return false;
    return read_frame(data, CMD_USER_INFO, LEN_USER_INFO);
}

bool ScudBmsProtocol::query_0x31(std::vector<uint8_t>& data) {
    if (!send_query(CMD_BATTERY_SUMMARY)) return false;
    return read_frame(data, CMD_BATTERY_SUMMARY, LEN_BATTERY_SUMMARY);
}

uint16_t ScudBmsProtocol::get_u16_be(const uint8_t* buf, int offset) {
    return static_cast<uint16_t>(buf[offset] << 8) | buf[offset + 1];
}

int16_t ScudBmsProtocol::get_i16_be(const uint8_t* buf, int offset) {
    return static_cast<int16_t>(get_u16_be(buf, offset));
}

uint16_t ScudBmsProtocol::get_u16_be(const std::vector<uint8_t>& buf,
                                     int offset) {
    return get_u16_be(buf.data(), offset);
}

int16_t ScudBmsProtocol::get_i16_be(const std::vector<uint8_t>& buf,
                                    int offset) {
    return get_i16_be(buf.data(), offset);
}

// One 0x61 query fills every field the protocol carries, or nothing at all:
// on failure status is left untouched, so the daemon publishes whole
// snapshots and a record's freshness always vouches for every field in it,
// power_on included. The protocol carries no HW/SW version, SOH or IO
// bitmap; zero them explicitly so the upper layer never sees a stale value.
//
// Cell extremes come from 0x61 (BYTE12/BYTE14). 0x31 is queried only when
// this 0x61 omitted them (both mV words zero) — a backfill for packs that
// do not report extremes in the user frame, mirroring the old rule that a
// healthy 0x61 must not be overwritten by 0x31. A failing 0x31 skips only
// the backfill and never fails the read.
bool ScudBmsProtocol::read_all(bms::BatteryStatus& status) {
    std::vector<uint8_t> d;
    if (!query_0x61(d)) return false;

    const uint16_t flag = get_u16_be(d, 0);
    const uint16_t chg_ma = get_u16_be(d, 2);
    const uint16_t dis_ma = get_u16_be(d, 4);
    const uint16_t max_mv = get_u16_be(d, 12);
    const uint16_t min_mv = get_u16_be(d, 14);

    status.voltage = get_u16_be(d, 10) / 1000.0; // output voltage, mV -> V
    // The protocol splits charge and discharge current into two fields; merge
    // them into one signed current: charging positive, discharging negative.
    const int mA = (chg_ma > 0) ? static_cast<int>(chg_ma)
                                : -static_cast<int>(dis_ma);
    status.current = mA / 1000.0;
    status.temperature = get_i16_be(d, 24);       // highest cell temp, degC
    status.max_cell_voltage = max_mv / 1000.0;
    status.min_cell_voltage = min_mv / 1000.0;
    status.protect_status = flag;                 // BAT FLAG, all 16 bits
    status.work_state = flag & 0x000F;            // bit0-3: enable + state flags
    status.cycles = get_u16_be(d, 26);            // cycle count

    status.percentage = get_u16_be(d, 20) / 100.0; // SOC % -> 0..1 fraction
    status.charge = get_u16_be(d, 18) / 1000.0;    // remaining Ah, from mAh
    status.capacity = get_u16_be(d, 16) / 1000.0;  // full capacity, from mAh
    status.design_capacity = status.capacity;
    status.soh = 0;

    status.power_on = (flag & 0x0001) ? 1 : 0;    // bit0 sys_dch_on
    status.io_state = 0;
    status.sw_version = 0;
    status.hw_version = 0;

    if (max_mv == 0 && min_mv == 0) {
        std::vector<uint8_t> cells;
        if (query_0x31(cells)) {
            // Recompute the extremes from the 15 cell voltages (unused slots
            // read 0 and are skipped).
            uint16_t mx = 0, mn = 0xFFFF;
            for (int i = 0; i < 15; i++) {
                const uint16_t v = get_u16_be(cells, i * 2);
                if (v == 0) continue;
                if (v > mx) mx = v;
                if (v < mn) mn = v;
            }
            if (mn != 0xFFFF) {
                status.max_cell_voltage = mx / 1000.0;
                status.min_cell_voltage = mn / 1000.0;
            }
        }
    }
    return true;
}

bool ScudBmsProtocol::read_version_info(bms::BatteryStatus& status) {
    std::vector<uint8_t> d;
    if (!query_0x61(d)) return false;

    status.cycles = get_u16_be(d, 26); // cycle count
    // The protocol carries no HW/SW version and no SOH; zero them so the upper
    // layer never prints uninitialized values.
    status.sw_version = 0;
    status.hw_version = 0;
    status.soh = 0;
    return true;
}

bool ScudBmsProtocol::read_serial_number(std::string&) {
    return false; // the protocol has no serial number command
}

bool ScudBmsProtocol::set_discharge_output(bool) {
    return false; // the protocol is read-only
}

} // namespace scud_bms

namespace {

// The daemon publishes one record per second and goes completely silent while
// every 485 read fails, so three missed records mean the bus behind the daemon
// is dead even though the Unix socket is still open.
constexpr double kDataStaleMs = 3000.0;

// CLOCK_MONOTONIC in microseconds, -1 when unavailable. The socket is local,
// so the value is directly comparable between daemon and client processes.
int64_t monotonic_us() {
    struct timespec ts {};
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return -1;
    return static_cast<int64_t>(ts.tv_sec) * 1000000LL +
           static_cast<int64_t>(ts.tv_nsec) / 1000LL;
}

} // namespace

ScudBmsDriver::ScudBmsDriver(const std::string& socket_path)
    : BmsDriver(), socket_path_(socket_path) {
    std::memset(&cached_, 0, sizeof(cached_));
    running_ = true;
    reader_thread_ = std::thread(&ScudBmsDriver::reader_loop, this);
    logger_->info("SCUD BMS driver started, socket={}", socket_path_);
}

ScudBmsDriver::~ScudBmsDriver() {
    running_ = false;
    if (reader_thread_.joinable()) {
        reader_thread_.join();
    }
    disconnect();
}

bool ScudBmsDriver::try_connect() {
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
    logger_->info("Connected to SCUD BMS daemon at {}", socket_path_);
    return true;
}

void ScudBmsDriver::disconnect() {
    if (sock_fd_ >= 0) {
        close(sock_fd_);
        sock_fd_ = -1;
    }
    connected_ = false;
    // A new socket must earn its freshness: the age of a previous session's
    // data must not survive the teardown, and an open socket that has not
    // delivered a record yet is not a battery the robot may act on.
    last_rx_us_ = -1;
}

void ScudBmsDriver::reader_loop() {
    pthread_setname_np(pthread_self(), "scud_bms_reader");

    bms::BatteryStatus wire {};
    size_t got = 0;

    while (running_) {
        if (!try_connect()) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        struct pollfd pfd {sock_fd_, POLLIN, 0};
        int ret = poll(&pfd, 1, 500);
        if (ret < 0) {
            if (errno == EINTR) continue;
            logger_->warn("SCUD BMS poll error, reconnecting...");
            disconnect();
            got = 0;
            continue;
        }
        if (ret == 0) {
            if (got > 0) {
                // A partial record that no further byte completes means the
                // stream is misaligned (a short write on the daemon side).
                // Drop it and reconnect: a fresh socket starts at a record
                // boundary, which is the only resync available without framing.
                logger_->warn("SCUD BMS stream misaligned at {} of {} bytes, "
                              "reconnecting...", got, sizeof(wire));
                disconnect();
                got = 0;
                continue;
            }
            // Nothing readable this round. Silence is exactly how a dead bus
            // looks (the daemon only writes when a read succeeded), so surface
            // the transition once instead of letting cached_ keep presenting
            // itself as current state.
            const double age = get_data_age_ms();
            if (age >= 0.0 && age > kDataStaleMs && !stale_logged_) {
                stale_logged_ = true;
                logger_->warn("SCUD BMS data stale: no record from the daemon "
                              "for {:.0f} ms (bus down or daemon idle)", age);
            }
            continue;
        }
        if (pfd.revents & (POLLERR | POLLHUP)) {
            logger_->warn("SCUD BMS daemon disconnected, reconnecting...");
            disconnect();
            got = 0;
            continue;
        }

        // SOCK_STREAM may short-read: accumulate a full BatteryStatus before
        // publishing it, otherwise a partial record would overwrite the cache
        ssize_t n = read(sock_fd_, reinterpret_cast<uint8_t*>(&wire) + got,
                         sizeof(wire) - got);
        if (n < 0) {
            if (errno == EINTR) continue;
            logger_->warn("SCUD BMS read error, reconnecting...");
            disconnect();
            got = 0;
            continue;
        }
        if (n == 0) {
            logger_->warn("SCUD BMS daemon disconnected, reconnecting...");
            disconnect();
            got = 0;
            continue;
        }

        got += static_cast<size_t>(n);
        if (got < sizeof(wire)) continue;

        {
            std::unique_lock lock(data_mutex_);
            cached_ = wire;
        }
        last_rx_us_ = monotonic_us();
        stale_logged_ = false;
        got = 0;
    }
}

double ScudBmsDriver::get_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.voltage;
}

double ScudBmsDriver::get_current() const {
    std::shared_lock lock(data_mutex_);
    return cached_.current;
}

double ScudBmsDriver::get_temperature() const {
    std::shared_lock lock(data_mutex_);
    return cached_.temperature;
}

double ScudBmsDriver::get_percentage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.percentage;
}

double ScudBmsDriver::get_charge() const {
    std::shared_lock lock(data_mutex_);
    return cached_.charge;
}

double ScudBmsDriver::get_capacity() const {
    std::shared_lock lock(data_mutex_);
    return cached_.capacity;
}

double ScudBmsDriver::get_design_capacity() const {
    std::shared_lock lock(data_mutex_);
    return cached_.design_capacity;
}

uint32_t ScudBmsDriver::get_protect_status() const {
    std::shared_lock lock(data_mutex_);
    return cached_.protect_status;
}

uint16_t ScudBmsDriver::get_work_state() const {
    std::shared_lock lock(data_mutex_);
    return cached_.work_state;
}

double ScudBmsDriver::get_max_cell_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.max_cell_voltage;
}

double ScudBmsDriver::get_min_cell_voltage() const {
    std::shared_lock lock(data_mutex_);
    return cached_.min_cell_voltage;
}

uint16_t ScudBmsDriver::get_soh() const {
    std::shared_lock lock(data_mutex_);
    return cached_.soh;
}

uint32_t ScudBmsDriver::get_cycles() const {
    std::shared_lock lock(data_mutex_);
    return cached_.cycles;
}

uint32_t ScudBmsDriver::get_io_state() const {
    std::shared_lock lock(data_mutex_);
    return cached_.io_state;
}

bool ScudBmsDriver::is_power_on() const {
    // An expired record must never be read back as the current hardware state:
    // this bit is what callers use to decide whether the pack is live.
    std::shared_lock lock(data_mutex_);
    return is_data_current() && cached_.power_on != 0;
}

bool ScudBmsDriver::is_connected() const {
    // Two different failures used to look identical here: the socket died, or
    // the socket is fine but the daemon stopped receiving anything from the
    // battery. Callers gate hardware actions on is_connected(), so both have to
    // read as "not connected"; get_data_age_ms() still tells them apart.
    return connected_.load() && is_data_current();
}

double ScudBmsDriver::get_data_age_ms() const {
    const int64_t rx = last_rx_us_.load();
    const int64_t now = monotonic_us();
    if (rx < 0 || now < 0) return -1.0; // nothing received yet / clock error
    if (now < rx) return 0.0;           // clock discontinuity: treat as current
    return static_cast<double>(now - rx) / 1000.0;
}

bool ScudBmsDriver::is_data_current() const {
    // Conservative by design: an unknown age counts as not current.
    const double age = get_data_age_ms();
    return age >= 0.0 && age <= kDataStaleMs;
}
