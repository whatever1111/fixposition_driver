/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Reconnect after the sensor closes a working TCP connection
 */

/* LIBC/STL */
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

/* EXTERNAL */
#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

/* PACKAGE */
#include "fixposition_driver_lib/fixposition_driver.hpp"

namespace {

const std::string CORRIMU_FRAME =
    "$FP,CORRIMU,2,2438,501103.343301,0.148529,-0.042478,9.816407,0.000228,0.000063,0.000091,1,1,*55\r\n";

// Hang protection only; receiving data or a connection wakes the waits immediately.
constexpr int WAIT_TIMEOUT_MS = 10000;

class TcpReconnectTest : public ::testing::Test {
   protected:
    void SetUp() override {
        listen_fd_ = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        ASSERT_GE(listen_fd_, 0) << std::strerror(errno);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ASSERT_EQ(bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)), 0) << std::strerror(errno);
        ASSERT_EQ(listen(listen_fd_, 1), 0) << std::strerror(errno);
        socklen_t address_size = sizeof(address);
        ASSERT_EQ(getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&address), &address_size), 0)
            << std::strerror(errno);

        fixposition::DriverParams params;
        params.stream_ = "tcpcli://127.0.0.1:" + std::to_string(ntohs(address.sin_port));
        params.reconnect_delay_ = 0.01;
        driver_ = std::make_unique<fixposition::FixpositionDriver>(params);
        driver_->AddRawObserver([this](const fpsdk::common::parser::ParserMsg& msg) {
            if (std::string(msg.data_.begin(), msg.data_.end()) == CORRIMU_FRAME) {
                std::lock_guard<std::mutex> lock(mutex_);
                ++frames_received_;
                received_.notify_one();
            }
        });
        ASSERT_TRUE(driver_->StartDriver());
    }

    void TearDown() override {
        // Join the worker before destroying any state captured by its observer.
        driver_.reset();
        if (sensor_fd_ >= 0) {
            close(sensor_fd_);
        }
        if (listen_fd_ >= 0) {
            close(listen_fd_);
        }
    }

    void AcceptSensorConnection() {
        pollfd pending{listen_fd_, POLLIN, 0};
        ASSERT_EQ(poll(&pending, 1, WAIT_TIMEOUT_MS), 1) << "Timed out waiting for a TCP connection";
        ASSERT_NE(pending.revents & POLLIN, 0);
        sensor_fd_ = accept4(listen_fd_, nullptr, nullptr, SOCK_NONBLOCK | SOCK_CLOEXEC);
        ASSERT_GE(sensor_fd_, 0) << std::strerror(errno);
    }

    void SendFrame() {
        ASSERT_EQ(send(sensor_fd_, CORRIMU_FRAME.data(), CORRIMU_FRAME.size(), MSG_DONTWAIT | MSG_NOSIGNAL),
                  static_cast<ssize_t>(CORRIMU_FRAME.size()))
            << std::strerror(errno);
    }

    bool WaitForFrames(const std::size_t count) {
        std::unique_lock<std::mutex> lock(mutex_);
        return received_.wait_for(lock, std::chrono::milliseconds(WAIT_TIMEOUT_MS),
                                  [this, count] { return frames_received_ >= count; });
    }

    int listen_fd_ = -1;
    int sensor_fd_ = -1;
    std::mutex mutex_;
    std::condition_variable received_;
    std::size_t frames_received_ = 0;  // Guarded by mutex_.
    std::unique_ptr<fixposition::FixpositionDriver> driver_;
};

TEST_F(TcpReconnectTest, ReconnectsAfterOrderlyPeerClose) {
    ASSERT_NO_FATAL_FAILURE(AcceptSensorConnection());
    ASSERT_NO_FATAL_FAILURE(SendFrame());
    ASSERT_TRUE(WaitForFrames(1)) << "The first connection did not deliver the frame";

    // Send FIN, not RST: the regression is recv() returning zero on an orderly close.
    ASSERT_EQ(shutdown(sensor_fd_, SHUT_WR), 0) << std::strerror(errno);
    ASSERT_EQ(close(sensor_fd_), 0) << std::strerror(errno);
    sensor_fd_ = -1;

    ASSERT_NO_FATAL_FAILURE(AcceptSensorConnection()) << "No second TCP connection after orderly peer close";
    ASSERT_NO_FATAL_FAILURE(SendFrame());
    ASSERT_TRUE(WaitForFrames(2)) << "The reconnected driver did not deliver the frame";
}

}  // namespace
