// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

/**
 * @file bms_daemon.cpp
 * @brief BMS daemon process that bridges serial Modbus BMS to a Unix domain socket.
 * @details Connects to the TWS BMS over serial port, continuously polls battery status,
 *          and exposes data via a Unix domain socket for client processes.
 */
 
#include <iostream>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>
#include <signal.h>
#include <sys/stat.h>
#include <cstring>
#include "bms_driver.hpp"

#define SOCKET_PATH "/tmp/bms.sock"

/* Global flag for graceful shutdown */
bool g_running = true;
void signal_handler(int sig) { g_running = false; }

int main(int argc, char** argv) {
    /* Register signal handlers for clean exit */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Check device name - robopi1 has no BMS */
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) == 0) {
        if (strcmp(hostname, "robopi1") == 0) {
            std::cout << "\033[1;34m[BMS Daemon] Device 'robopi1' detected. No BMS present. Entering sleep mode...\033[0m" << std::endl;
            while (g_running) {
                std::this_thread::sleep_for(std::chrono::hours(24));
            }
            return 0;
        }
    }

    /* Get arguments or use defaults */
    std::string port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    int baud = (argc > 2) ? std::stoi(argv[2]) : 115200;
    int timeout = (argc > 3) ? std::stoi(argv[3]) : 300;

    tws_bms::BmsProtocol bms_proto(port, baud, timeout);

    /* Keep trying to open the serial port */
    while (g_running && !bms_proto.open()) {
        std::cerr << "\033[1;33m[BMS Daemon] Waiting for serial port " << port << "...\033[0m" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    if (!g_running) return 0;

    /* Print Static Info at Startup */
    bms::BatteryStatus static_info;
    if (bms_proto.read_version_info(static_info)) {
        std::cout << "\033[1;32m[BMS Daemon] Connected to BMS.\033[0m" << std::endl;
        std::cout << " > FW Version: 0x" << std::hex << static_info.sw_version << " | HW Version: 0x" << static_info.hw_version << std::dec << std::endl;
        std::cout << " > Health (SOH): " << static_info.soh << "% | Cycles: " << static_info.cycles << std::endl;
    }

    /* Initialize Unix Domain Socket Server */
    int server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "[BMS Daemon] Failed to create socket" << std::endl;
        return 1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
    
    /* Cleanup existing socket file */
    unlink(SOCKET_PATH);
    
    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[BMS Daemon] Bind failed" << std::endl;
        return 1;
    }

    if (listen(server_fd, 5) < 0) {
        std::cerr << "[BMS Daemon] Listen failed" << std::endl;
        return 1;
    }

    /* Set socket permissions so ROS node can connect */
    chmod(SOCKET_PATH, 0666);

    std::cout << "[BMS Daemon] Started. Heartbeat Active on " << port << std::endl;

    bms::BatteryStatus status_to_send;
    memset(&status_to_send, 0, sizeof(status_to_send));
    
    std::vector<int> clients;
    int failure_count = 0;

    while (g_running) {
        /* 1. Fetch data from BMS (This acts as the heartbeat) */
        bms::BatteryStatus raw_data;
        bool ok_basic = bms_proto.read_basic_info(raw_data);
        usleep(50000); // Wait 50ms between requests to avoid bus congestion
        bool ok_capacity = bms_proto.read_capacity_info(raw_data);

        if (ok_basic || ok_capacity) {
            failure_count = 0; // Reset failure counter on any successful read
            
            /* 2. Map raw data to shared POD structure (Partial update is better than no update) */
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

            std::cout << "[BMS Data] Voltage: " << status_to_send.voltage << "V | "
                      << "Current: " << status_to_send.current << "A | "
                      << "SoC: " << status_to_send.percentage * 100.0 << "%" << std::endl;
            // ------------------
            
            /* 3. Broadcast to all connected clients */
            for (auto it = clients.begin(); it != clients.end(); ) {
                if (write(*it, &status_to_send, sizeof(status_to_send)) < 0) {
                    close(*it);
                    it = clients.erase(it);
                } else {
                    ++it;
                }
            }
        } else {
            failure_count++;
            std::cerr << "[BMS Daemon] BMS Read Failure (" << failure_count << "/5)" << std::endl;
            
            if (failure_count >= 5) {
                std::cerr << "[BMS Daemon] Port seems disconnected. Re-opening..." << std::endl;
                bms_proto.close_port();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                bms_proto.open();
                failure_count = 0;
            }
        }

        /* 4. Accept new client connections (Non-blocking) */
        struct timeval tv = {0, 10000}; /* 10ms timeout */
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(server_fd, &rfds);
        if (select(server_fd + 1, &rfds, NULL, NULL, &tv) > 0) {
            int cfd = accept(server_fd, NULL, NULL);
            if (cfd >= 0) {
                clients.push_back(cfd);
                std::cout << "[BMS Daemon] New client connected." << std::endl;
            }
        }
        
        /* Loop frequency ~1Hz */
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    /* Cleanup */
    for (int c : clients) close(c);
    close(server_fd);
    unlink(SOCKET_PATH);
    std::cout << "[BMS Daemon] Shutdown complete." << std::endl;
    return 0;
}
