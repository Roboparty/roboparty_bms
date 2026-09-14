// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include <iostream>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>
#include <csignal>
#include <sys/stat.h>
#include <cstring>
#include <type_traits>
#include "bms_driver.hpp"
#include "gfcan_bms_driver.hpp"
#include "nrf_pmic_driver.hpp"

static bool g_running = true;
static void signal_handler(int) { g_running = false; }

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
                       const std::string& socket_path) {
    while (g_running && !proto.open()) {
        std::cerr << "[BMS Daemon] Waiting for serial port " << port << "..."
                  << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    if (!g_running) return;

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

    if (type == "GFCAN") {
        std::cout << "[BMS Daemon] Type=" << type << " Port=" << port << std::endl;
        std::string socket_path = (argc > 2) ? argv[2] : "/tmp/can_bms.sock";
        run_can_daemon(port, socket_path);
        return 0;
    }

    int baud = (argc > 2) ? std::stoi(argv[2], nullptr, 0) : 115200;
    int timeout = (argc > 3) ? std::stoi(argv[3], nullptr, 0) : 300;

    std::cout << "[BMS Daemon] Type=" << type << " Port=" << port
              << " Baud=" << baud << " Timeout=" << timeout << std::endl;

    if (type == "TWS") {
        uint8_t dev_addr = (argc > 5) ? static_cast<uint8_t>(std::stoi(argv[5], nullptr, 0))
                                       : 0x01;
        tws_bms::BmsProtocol proto(port, baud, timeout, dev_addr);
        run_daemon(proto, port, "/tmp/bms.sock");
    } else if (type == "GF") {
        uint8_t dev_addr = (argc > 5) ? static_cast<uint8_t>(std::stoi(argv[5], nullptr, 0))
                                       : 0x03;
        gf_bms::GfBmsProtocol proto(port, baud, timeout, dev_addr);
        run_daemon(proto, port, "/tmp/gf_bms.sock");
    } else if (type == "SCUD485") {
        // Point-to-point frames (no device address). The spec mandates 19200
        // 8N1, so BAUD_RATE must be set to 19200 in /etc/default/bms_daemon.
        scud_bms::ScudBmsProtocol proto(port, baud, timeout);
        run_daemon(proto, port, "/tmp/bms.sock");
    } else if (type == "GFCAN") {
        std::string socket_path = (argc > 2) ? argv[2] : "/tmp/can_bms.sock";
        run_can_daemon(port, socket_path);
    } else if (type == "NRF") {
        auto driver = std::make_shared<NrfPmicDriver>(port);
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
