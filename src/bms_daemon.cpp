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
#include "bms_driver.hpp"
#include "gfcan_bms_driver.hpp"
#include "nrf_pmic_driver.hpp"

static bool g_running = true;
static void signal_handler(int) { g_running = false; }

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
        bms::BatteryStatus raw_data;
        bool ok_basic = proto.read_basic_info(raw_data);
        usleep(50000);
        bool ok_capacity = proto.read_capacity_info(raw_data);

        if (ok_basic || ok_capacity) {
            failure_count = 0;

            if (ok_basic) {
                status_to_send.voltage = raw_data.voltage;
                status_to_send.current = raw_data.current;
                status_to_send.temperature = raw_data.temperature;
                status_to_send.protect_status = raw_data.protect_status;
                status_to_send.work_state = raw_data.work_state;
                status_to_send.max_cell_voltage = raw_data.max_cell_voltage;
                status_to_send.min_cell_voltage = raw_data.min_cell_voltage;
            }
            if (ok_capacity) {
                status_to_send.percentage = raw_data.percentage;
                status_to_send.charge = raw_data.charge;
                status_to_send.capacity = raw_data.capacity;
                status_to_send.soh = raw_data.soh;
            }

            std::cout << "[BMS Data] Voltage: " << status_to_send.voltage
                      << "V | Current: " << status_to_send.current
                      << "A | SoC: " << status_to_send.percentage * 100.0
                      << "%" << std::endl;

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

    /* Check device name - robopi1 has no BMS */
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) == 0) {
        if (strcmp(hostname, "robopi1") == 0) {
            std::cout << "[BMS Daemon] Device 'robopi1' detected. "
                      << "No BMS present. Entering sleep mode..." << std::endl;
            while (g_running) {
                std::this_thread::sleep_for(std::chrono::hours(24));
            }
            return 0;
        }
    }

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

    int baud = (argc > 2) ? std::stoi(argv[2]) : 115200;
    int timeout = (argc > 3) ? std::stoi(argv[3]) : 300;

    std::cout << "[BMS Daemon] Type=" << type << " Port=" << port
              << " Baud=" << baud << " Timeout=" << timeout << std::endl;

    if (type == "TWS") {
        uint8_t dev_addr = (argc > 5) ? static_cast<uint8_t>(std::stoi(argv[5]))
                                       : 0x01;
        tws_bms::BmsProtocol proto(port, baud, timeout, dev_addr);
        run_daemon(proto, port, "/tmp/bms.sock");
    } else if (type == "GF") {
        uint8_t dev_addr = (argc > 5) ? static_cast<uint8_t>(std::stoi(argv[5]))
                                       : 0x03;
        gf_bms::GfBmsProtocol proto(port, baud, timeout, dev_addr);
        run_daemon(proto, port, "/tmp/gf_bms.sock");
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
