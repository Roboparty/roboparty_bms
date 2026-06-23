// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2026 wentywenty

#include "socket_can.hpp"

#include <cstdio>
#include <errno.h>

std::unordered_map<std::string, std::shared_ptr<SocketCAN>> SocketCAN::instances_;

std::shared_ptr<SocketCAN> SocketCAN::get_instance(const std::string& iface) {
    if (instances_.find(iface) == instances_.end()) {
        instances_[iface] = std::shared_ptr<SocketCAN>(new SocketCAN(iface));
    }
    return instances_[iface];
}

SocketCAN::SocketCAN(const std::string& iface) : iface_(iface) {
    open(iface);
}

SocketCAN::~SocketCAN() {
    close();
}

void SocketCAN::open(const std::string& iface) {
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ < 0) {
        throw std::runtime_error("Failed to create CAN socket");
    }

    struct ifreq ifr {};
    strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
    if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
        close();
        throw std::runtime_error("CAN interface not found: " + iface);
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sockfd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close();
        throw std::runtime_error("Failed to bind to CAN interface: " + iface);
    }

    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags < 0 || fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        close();
        throw std::runtime_error("Failed to set non-blocking on CAN socket");
    }

    receiving_ = true;
    receiver_thread_ = std::thread(&SocketCAN::receiver_loop, this);
}

void SocketCAN::close() {
    receiving_ = false;
    if (receiver_thread_.joinable()) {
        receiver_thread_.join();
    }
    if (sockfd_ >= 0) {
        ::close(sockfd_);
        sockfd_ = CAN_FD_INIT;
    }
}

void SocketCAN::receiver_loop() {
    pthread_setname_np(pthread_self(), "can_rx");

    fd_set fds;
    struct timeval tv;
    can_frame rx_frame;

    while (receiving_) {
        FD_ZERO(&fds);
        FD_SET(sockfd_, &fds);

        tv.tv_sec = 0;
        tv.tv_usec = CAN_TIMEOUT_USEC;

        int ret = select(sockfd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (ret == 0) continue;

        while (true) {
            ssize_t n = read(sockfd_, &rx_frame, sizeof(rx_frame));
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                break;
            }
            if (n == 0) break;

            CanCallback cb;
            {
                std::lock_guard<std::mutex> lock(callback_mutex_);
                CanId id = id_extractor_(rx_frame);
                auto it = callbacks_.find(id);
                if (it != callbacks_.end()) {
                    cb = it->second;
                }
            }
            if (cb) {
                cb(rx_frame);
            }
        }
    }
}

void SocketCAN::add_callback(CanId id, CanCallback cb) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_[id] = std::move(cb);
}

void SocketCAN::remove_callback(CanId id) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_.erase(id);
}

void SocketCAN::clear_callbacks() {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_.clear();
}

void SocketCAN::set_id_extractor(CanIdExtractor extractor) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    id_extractor_ = std::move(extractor);
}
