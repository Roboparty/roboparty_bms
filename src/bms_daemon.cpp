// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include <iostream>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <pthread.h>
#include <stdexcept>
#include <sys/stat.h>
#include <type_traits>
#include "bms_driver.hpp"
#include "gfcan_bms_driver.hpp"
#include "nrf_pmic_driver.hpp"

// Written by the signal handler, read by the main loop and the forward pump
// thread, so it has to be atomic rather than a plain bool.
static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// ---------------------------------------------------------------------------
// UART forwarding
//
// bms_daemon owns the BMS serial port, so it is the only process that may read
// it. When forwarding is on it also fans every byte it reads out to a second
// UART. This replaces robopi-addon's robopi-uart-bridge, which ran as a
// separate process reading the same tty: two readers on one tty steal bytes
// from each other, so the daemon saw intermittent Modbus read failures and the
// bridge only ever saw a partial stream. One reader tapping its own feed gives
// the downstream port the complete stream at no cost to the BMS.
//
// When the configured source is the BMS port itself the tap hangs off the
// protocol's SerialPort. Otherwise (BMS on CAN, or on a different UART) the
// source is opened here and pumped by a thread -- same tap, other driver.
// ---------------------------------------------------------------------------

struct ForwardConfig {
    bool enabled = true;
    std::string port_a = "/dev/ttyS3";
    std::string port_b = "/dev/ttyS7";
    int baud = 115200;
};

static std::string env_or(const char* name, const std::string& fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') return fallback;
    return std::string(value);
}

static int env_int(const char* name, int fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') return fallback;
    try {
        return std::stoi(value, nullptr, 0);
    } catch (const std::exception&) {
        std::cerr << "[BMS Daemon] " << name << "=" << value
                  << " is not a number; using " << fallback << std::endl;
        return fallback;
    }
}

static bool env_flag(const char* name, bool fallback) {
    const char* value = std::getenv(name);
    if (value == nullptr || *value == '\0') return fallback;
    std::string text(value);
    for (char& c : text) c = static_cast<char>(std::tolower(c));
    if (text == "0" || text == "false" || text == "no" || text == "off") return false;
    if (text == "1" || text == "true" || text == "yes" || text == "on") return true;
    std::cerr << "[BMS Daemon] " << name << "=" << value
              << " is not a recognized boolean; using "
              << (fallback ? "enabled" : "disabled") << std::endl;
    return fallback;
}

// SerialPort falls back to 9600 for anything it does not recognize, so a typo
// here would silently forward at the wrong speed.
static bool baud_supported(int baud) {
    switch (baud) {
    case 9600: case 19200: case 38400: case 57600:
    case 115200: case 230400: case 460800: case 921600:
        return true;
    default:
        return false;
    }
}

// True when two paths name the same character device. This decides whether
// the forwarder taps the protocol's own port or opens a second one, so it has
// to see through symlinks such as /dev/serial/by-id/... -- a plain string
// compare would miss those and quietly reintroduce a second reader on the bus.
static bool same_device(const std::string& a, const std::string& b) {
    if (a.empty() || b.empty()) return false;
    if (a == b) return true;
    struct stat sa {};
    struct stat sb {};
    if (stat(a.c_str(), &sa) == 0 && stat(b.c_str(), &sb) == 0) {
        return S_ISCHR(sa.st_mode) && S_ISCHR(sb.st_mode) &&
               sa.st_rdev == sb.st_rdev;
    }
    return false;
}

class UartForwarder {
   public:
    UartForwarder(const ForwardConfig& cfg, bool standalone)
        : cfg_(cfg), standalone_(standalone) {}

    ~UartForwarder() { stop(); }

    UartForwarder(const UartForwarder&) = delete;
    UartForwarder& operator=(const UartForwarder&) = delete;

    // Where the tapped bytes go, for the case where the source is the BMS
    // port the protocol already reads. Must be set before the first poll.
    void set_tap(std::function<void(int)> tap) { tap_ = std::move(tap); }

    void start() {
        if (!standalone_) return;
        running_ = true;
        thread_ = std::thread(&UartForwarder::pump_loop, this);
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        detach_tap();
        source_.close();
        sink_.close();
    }

    // Opens the destination when it appears and (re)attaches the tap exactly
    // once per open. Must be called periodically by the owner in tap mode; in
    // standalone mode the pump thread drives it.
    void poll_sink() {
        if (!sink_.is_open()) {
            detach_tap();
            if (cfg_.port_b.empty()) return;
            if (!sink_.open(cfg_.port_b, cfg_.baud)) {
                if (!sink_warned_) {
                    std::cerr << "[BMS Daemon] Forward destination "
                              << cfg_.port_b << " unavailable; retrying"
                              << std::endl;
                    sink_warned_ = true;
                }
                return;
            }
            sink_warned_ = false;
            std::cout << "[BMS Daemon] Forwarding "
                      << (standalone_ ? cfg_.port_a : std::string("BMS bus"))
                      << " -> " << cfg_.port_b << " at " << cfg_.baud
                      << std::endl;
        }
        if (attached_ != sink_.fd()) {
            if (tap_) {
                tap_(sink_.fd());
            } else {
                source_.set_forward_fd(sink_.fd());
            }
            attached_ = sink_.fd();
        }
    }

   private:
    // Hands the tap back with -1 so a stale descriptor is never written to
    // after the destination is closed or reopened.
    void detach_tap() {
        if (attached_ == -1) return;
        if (tap_) {
            tap_(-1);
        } else {
            source_.set_forward_fd(-1);
        }
        attached_ = -1;
    }

    void pump_loop() {
        pthread_setname_np(pthread_self(), "bms_forward");
        uint8_t buf[4096];
        while (running_) {
            poll_sink();
            if (!source_.is_open()) {
                if (!source_.open(cfg_.port_a, cfg_.baud)) {
                    if (!source_warned_) {
                        std::cerr << "[BMS Daemon] Forward source "
                                  << cfg_.port_a << " unavailable; retrying"
                                  << std::endl;
                        source_warned_ = true;
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    continue;
                }
                source_warned_ = false;
                // Re-attach: the descriptor below may be a fresh one.
                attached_ = -1;
                std::cout << "[BMS Daemon] Forward source " << cfg_.port_a
                          << " opened" << std::endl;
            }

            // read_raw() polls and taps internally; the bytes read here are
            // consumed only to keep the tap running.
            ssize_t n = source_.read_raw(buf, sizeof(buf), 200);
            if (n < 0 && errno != EAGAIN && errno != EINTR) {
                source_.close();
            }
        }
    }

    ForwardConfig cfg_;
    bool standalone_ = false;
    std::function<void(int)> tap_;

    bms::SerialPort source_;
    bms::SerialPort sink_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    int attached_ = -1;
    bool sink_warned_ = false;
    bool source_warned_ = false;
};

// Protocols that can fill a complete BatteryStatus from a single bus
// transaction publish whole snapshots: a record on the wire then vouches for
// every field in it, and nothing is carried over from an older round.
// Everything else keeps the per-field merge loop below. Adding a protocol to
// this list is a deliberate switch of its broadcast semantics.
template <typename P>
struct publishes_snapshots : std::false_type {};

template <>
struct publishes_snapshots<scud_bms::ScudBmsProtocol> : std::true_type {};

static_assert(publishes_snapshots<scud_bms::ScudBmsProtocol>::value,
              "SCUD485 must publish snapshot records");
static_assert(!publishes_snapshots<tws_bms::BmsProtocol>::value,
              "TWS keeps the per-field carry-forward loop");
static_assert(!publishes_snapshots<gf_bms::GfBmsProtocol>::value,
              "GF keeps the per-field carry-forward loop");

template <typename Protocol>
static void run_daemon(Protocol& proto, const std::string& port,
                       const std::string& socket_path,
                       UartForwarder* forwarder = nullptr) {
    while (g_running && !proto.open()) {
        std::cerr << "[BMS Daemon] Waiting for serial port " << port << "..."
                  << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    if (!g_running) return;

    // Attach the forwarding tap before the first read so no bus traffic is
    // missed. A destination that is not up yet is fine: poll_sink() retries
    // from the main loop and the tap simply stays detached until then.
    if (forwarder) forwarder->poll_sink();

    /* Print Static Info at Startup */
    bms::BatteryStatus static_info;
    if (proto.read_version_info(static_info)) {
        std::cout << "[BMS Daemon] Connected to BMS." << std::endl;
        std::cout << " > FW Version: 0x" << std::hex << static_info.sw_version
                  << " | HW Version: 0x" << static_info.hw_version << std::dec
                  << std::endl;
        std::cout << " > Health (SOH): " << static_info.soh
                  << "% | Cycles: " << static_info.cycles << std::endl;
    }

    /* Initialize Unix Domain Socket Server */
    int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "[BMS Daemon] Failed to create socket" << std::endl;
        return;
    }

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

    unlink(socket_path.c_str());

    if (bind(server_fd, reinterpret_cast<struct sockaddr*>(&addr),
             sizeof(addr)) < 0) {
        std::cerr << "[BMS Daemon] Bind failed" << std::endl;
        return;
    }

    if (listen(server_fd, 5) < 0) {
        std::cerr << "[BMS Daemon] Listen failed" << std::endl;
        return;
    }

    chmod(socket_path.c_str(), 0666);

    std::cout << "[BMS Daemon] Started. Heartbeat Active on " << port
              << std::endl;

    bms::BatteryStatus status_to_send;
    memset(&status_to_send, 0, sizeof(status_to_send));

    std::vector<int> clients;
    int failure_count = 0;

    while (g_running) {
        // Re-checks the destination each round: it may have appeared late, or
        // been reopened, in which case the tap is re-pointed at the new fd.
        if (forwarder) forwarder->poll_sink();

        bms::BatteryStatus raw_data{};
        bool ok_read = false;
        // Whether power_on in this round's broadcast is fresh: the legacy
        // loop only knows that after a successful IO-state read, the
        // snapshot loop always does.
        bool show_power = false;

        if constexpr (publishes_snapshots<Protocol>::value) {
            // One transaction fills every field or none, so the published
            // snapshot can never mix rounds.
            ok_read = proto.read_all(raw_data);
            if (ok_read) {
                status_to_send = raw_data;
                show_power = true;
            }
        } else {
            bool ok_basic = proto.read_basic_info(raw_data);
            usleep(50000);
            bool ok_capacity = proto.read_capacity_info(raw_data);
            bool ok_io_state = proto.read_io_state(raw_data);
            ok_read = ok_basic || ok_capacity || ok_io_state;
            show_power = ok_io_state;

            if (ok_read) {
                if (ok_basic) {
                    status_to_send.voltage = raw_data.voltage;
                    status_to_send.current = raw_data.current;
                    status_to_send.temperature = raw_data.temperature;
                    status_to_send.protect_status = raw_data.protect_status;
                    status_to_send.work_state = raw_data.work_state;
                    status_to_send.max_cell_voltage = raw_data.max_cell_voltage;
                    status_to_send.min_cell_voltage = raw_data.min_cell_voltage;
                    status_to_send.cycles = raw_data.cycles;
                }
                if (ok_capacity) {
                    status_to_send.percentage = raw_data.percentage;
                    status_to_send.charge = raw_data.charge;
                    status_to_send.capacity = raw_data.capacity;
                    status_to_send.design_capacity = raw_data.design_capacity;
                    status_to_send.soh = raw_data.soh;
                }
                if (ok_io_state) {
                    status_to_send.io_state = raw_data.io_state;
                    status_to_send.power_on = raw_data.power_on;
                    // 0x31 cell-voltage backfill: forward only when the basic
                    // read failed, so it cannot overwrite valid 0x61 extremes
                    if (!ok_basic && raw_data.max_cell_voltage > 0.0) {
                        status_to_send.max_cell_voltage =
                            raw_data.max_cell_voltage;
                        status_to_send.min_cell_voltage =
                            raw_data.min_cell_voltage;
                    }
                }
            }
        }

        if (ok_read) {
            failure_count = 0;

            std::cout << "[BMS Data] Voltage: " << status_to_send.voltage
                      << "V | Current: " << status_to_send.current
                      << "A | SoC: " << status_to_send.percentage * 100.0
                      << "%";
            if (show_power) {
                std::cout << " | Power: "
                          << (status_to_send.power_on ? "ON" : "OFF");
            }
            std::cout << std::endl;

            for (auto it = clients.begin(); it != clients.end();) {
                if (write(*it, &status_to_send, sizeof(status_to_send)) < 0) {
                    close(*it);
                    it = clients.erase(it);
                } else {
                    ++it;
                }
            }
        } else {
            failure_count++;
            std::cerr << "[BMS Daemon] BMS Read Failure (" << failure_count
                      << "/5)" << std::endl;

            if (failure_count >= 5) {
                std::cerr << "[BMS Daemon] Port seems disconnected. "
                          << "Re-opening..." << std::endl;
                proto.close_port();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                proto.open();
                failure_count = 0;
            }
        }

        /* Accept new client connections (Non-blocking) */
        struct timeval tv = {0, 10000};
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(server_fd, &rfds);
        if (select(server_fd + 1, &rfds, nullptr, nullptr, &tv) > 0) {
            int cfd = accept(server_fd, nullptr, nullptr);
            if (cfd >= 0) {
                clients.push_back(cfd);
                std::cout << "[BMS Daemon] New client connected." << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    for (int c : clients) close(c);
    close(server_fd);
    unlink(socket_path.c_str());
    std::cout << "[BMS Daemon] Shutdown complete." << std::endl;
}

static void run_can_daemon(const std::string& can_iface,
                           const std::string& socket_path) {
    auto driver = std::make_unique<bms::GfCanBmsDriver>(can_iface);

    bms::BatteryStatus status{};
    size_t stale_iters = 0;

    while (g_running && stale_iters < 50) {
        status.voltage = driver->get_voltage();
        if (status.voltage > 0.0) {
            stale_iters = 0;
            status.current = driver->get_current();
            status.temperature = driver->get_temperature();
            status.percentage = driver->get_percentage();
            status.charge = driver->get_charge();
            status.capacity = driver->get_capacity();
            status.design_capacity = driver->get_design_capacity();
            status.protect_status = driver->get_protect_status();
            status.work_state = driver->get_work_state();
            status.max_cell_voltage = driver->get_max_cell_voltage();
            status.min_cell_voltage = driver->get_min_cell_voltage();
            status.soh = driver->get_soh();
            status.cycles = driver->get_cycles();
        } else {
            stale_iters++;
            std::cerr << "[CAN BMS Daemon] Waiting for BMS data..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        std::cout << "[CAN BMS Data] Voltage: " << status.voltage
                  << "V | Current: " << status.current
                  << "A | SoC: " << status.percentage << "%" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "[CAN BMS Daemon] Shutdown complete." << std::endl;
}

// Builds the forwarder for a serial protocol. When the configured source is
// the port the protocol already reads, the tap hangs off that very port, so
// the bus is still opened exactly once. Otherwise the source is opened and
// pumped on its own thread.
template <typename Protocol>
static std::unique_ptr<UartForwarder> make_protocol_forwarder(
        const ForwardConfig& cfg, Protocol& proto, const std::string& bms_port) {
    const bool same_port = same_device(cfg.port_a, bms_port);
    auto forwarder = std::make_unique<UartForwarder>(cfg, !same_port);
    if (same_port) {
        forwarder->set_tap([&proto](int fd) { proto.set_forward_fd(fd); });
    } else {
        forwarder->start();
    }
    return forwarder;
}

// For BMS types that own no serial port at all (GFCAN, NRF): the forward
// source is always a port of its own.
static std::unique_ptr<UartForwarder> make_standalone_forwarder(
        const ForwardConfig& cfg) {
    auto forwarder = std::make_unique<UartForwarder>(cfg, true);
    forwarder->start();
    return forwarder;
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Get arguments or use defaults */
    std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    std::string type = "TWS";
    if (argc > 4) {
        type = argv[4];
    } else if (argc > 3) {
        std::string arg3(argv[3]);
        if (arg3 == "GFCAN" || arg3 == "NRF") type = arg3;
    }

    /* UART forwarding, inherited from robopi-addon's robopi-uart-bridge.
       Absent keys keep the previous behaviour: forwarding on, ttyS3 -> ttyS7.
       The old bridge was enabled on every host, so defaulting to off here
       would silently stop forwarding on machines that rely on it. */
    ForwardConfig forward;
    forward.enabled = env_flag("FORWARD_ENABLE", true);
    forward.port_a = env_or("FORWARD_PORT_A", "/dev/ttyS3");
    forward.port_b = env_or("FORWARD_PORT_B", "/dev/ttyS7");
    forward.baud = env_int("FORWARD_BAUD", 115200);
    if (forward.enabled && !baud_supported(forward.baud)) {
        std::cerr << "[BMS Daemon] FORWARD_BAUD=" << forward.baud
                  << " is unsupported; the serial layer falls back to 9600"
                  << std::endl;
    }

    if (type == "GFCAN") {
        std::cout << "[BMS Daemon] Type=" << type << " Port=" << port << std::endl;
        std::string socket_path = (argc > 2) ? argv[2] : "/tmp/can_bms.sock";
        // CAN owns no serial port, so the forward source is always its own.
        auto forwarder =
            forward.enabled ? make_standalone_forwarder(forward) : nullptr;
        run_can_daemon(port, socket_path);
        return 0;
    }

    int baud = (argc > 2) ? std::stoi(argv[2], nullptr, 0) : 115200;
    int timeout = (argc > 3) ? std::stoi(argv[3], nullptr, 0) : 300;

    std::cout << "[BMS Daemon] Type=" << type << " Port=" << port
              << " Baud=" << baud << " Timeout=" << timeout << std::endl;

    // The source is read at BAUD_RATE while the sink runs at FORWARD_BAUD.
    // Both default to 115200, but a host whose bus runs at another rate (the
    // SCUD485 spec mandates 19200) would forward into a UART transmitting at
    // the wrong speed and produce a plausible-looking but garbled stream.
    if (forward.enabled && forward.baud != baud &&
        same_device(forward.port_a, port)) {
        std::cerr << "[BMS Daemon] FORWARD_BAUD=" << forward.baud
                  << " differs from BAUD_RATE=" << baud
                  << "; the forwarded stream is only valid if the downstream "
                     "port expects " << forward.baud << std::endl;
    }

    if (type == "TWS") {
        uint8_t dev_addr = (argc > 5) ? static_cast<uint8_t>(std::stoi(argv[5], nullptr, 0))
                                       : 0x01;
        tws_bms::BmsProtocol proto(port, baud, timeout, dev_addr);
        // Declared after proto so the tap outlives nothing it points at.
        auto forwarder = forward.enabled
                             ? make_protocol_forwarder(forward, proto, port)
                             : nullptr;
        run_daemon(proto, port, "/tmp/bms.sock", forwarder.get());
    } else if (type == "GF") {
        uint8_t dev_addr = (argc > 5) ? static_cast<uint8_t>(std::stoi(argv[5], nullptr, 0))
                                       : 0x03;
        gf_bms::GfBmsProtocol proto(port, baud, timeout, dev_addr);
        auto forwarder = forward.enabled
                             ? make_protocol_forwarder(forward, proto, port)
                             : nullptr;
        run_daemon(proto, port, "/tmp/gf_bms.sock", forwarder.get());
    } else if (type == "SCUD485") {
        // Point-to-point frames (no device address). The spec mandates 19200
        // 8N1, so BAUD_RATE must be set to 19200 in /etc/default/bms_daemon.
        scud_bms::ScudBmsProtocol proto(port, baud, timeout);
        auto forwarder = forward.enabled
                             ? make_protocol_forwarder(forward, proto, port)
                             : nullptr;
        run_daemon(proto, port, "/tmp/bms.sock", forwarder.get());
    } else if (type == "GFCAN") {
        std::string socket_path = (argc > 2) ? argv[2] : "/tmp/can_bms.sock";
        auto forwarder =
            forward.enabled ? make_standalone_forwarder(forward) : nullptr;
        run_can_daemon(port, socket_path);
    } else if (type == "NRF") {
        auto driver = std::make_shared<NrfPmicDriver>(port);
        auto forwarder =
            forward.enabled ? make_standalone_forwarder(forward) : nullptr;
        while (g_running) {
            auto status = driver->status();
            std::cout << "[NRF PMIC] " << status.variant.name
                      << " | 48V=" << (status.power48 ? "ON" : "OFF")
                      << " 5V=" << (status.power5 ? "ON" : "OFF")
                      << " | BMS " << status.bms.voltage << "V "
                      << status.bms.current << "A "
                      << status.bms.soc << "%" << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    } else {
        std::cerr << "[BMS Daemon] Unknown BMS type: " << type << std::endl;
        return 1;
    }

    return 0;
}
