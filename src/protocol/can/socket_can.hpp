// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <atomic>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

constexpr int CAN_FD_INIT = -1;
constexpr int CAN_TIMEOUT_USEC = 1000;

using CanCallback = std::function<void(const can_frame&)>;
using CanId = uint32_t;
using CanCallbackMap = std::unordered_map<CanId, CanCallback>;
using CanIdExtractor = std::function<CanId(const can_frame&)>;

class SocketCAN {
public:
    SocketCAN(const SocketCAN&) = delete;
    SocketCAN& operator=(const SocketCAN&) = delete;
    ~SocketCAN();

    static std::shared_ptr<SocketCAN> get_instance(const std::string& iface);

    void add_callback(CanId id, CanCallback cb);
    void remove_callback(CanId id);
    void clear_callbacks();
    void set_id_extractor(CanIdExtractor extractor);

private:
    explicit SocketCAN(const std::string& iface);

    void open(const std::string& iface);
    void close();
    void receiver_loop();

    std::string iface_;
    int sockfd_ = CAN_FD_INIT;
    std::atomic<bool> receiving_{false};
    std::thread receiver_thread_;

    CanCallbackMap callbacks_;
    std::mutex callback_mutex_;
    CanIdExtractor id_extractor_ = [](const can_frame& f) -> CanId {
        return (f.can_id & CAN_EFF_FLAG) ? (f.can_id & CAN_EFF_MASK) : (f.can_id & CAN_SFF_MASK);
    };

    static std::unordered_map<std::string, std::shared_ptr<SocketCAN>> instances_;
};
