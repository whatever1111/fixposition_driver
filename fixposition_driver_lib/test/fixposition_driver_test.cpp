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
 * @brief Fixposition sensor driver: tests for reading the sensor connection
 */

/* LIBC/STL */
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

/* EXTERNAL */
#include <arpa/inet.h>
#include <fcntl.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

/* PACKAGE */
#include "fixposition_driver_lib/fixposition_driver.hpp"

namespace {
/* ****************************************************************************************************************** */

using fixposition::DriverParams;
using fixposition::FixpositionDriver;

// The production driver, with its protected connection methods made callable. Nothing is overridden.
class TestDriver : public FixpositionDriver {
   public:
    using FixpositionDriver::Connect;
    using FixpositionDriver::Disconnect;
    using FixpositionDriver::FixpositionDriver;
    using FixpositionDriver::IsConnected;
    using FixpositionDriver::Read;
};

// One FP_A-CORRIMU frame as a VRTK2 sent it over TCP: 97 bytes, 200 of them a second
const std::string CORRIMU_FRAME =
    "$FP,CORRIMU,2,2438,501103.343301,0.148529,-0.042478,9.816407,0.000228,0.000063,0.000091,1,1,*55\r\n";

// Only bounds a failing test: data that has arrived is returned at once
constexpr int READ_TIMEOUT_MS = 5000;

// Fill byte for the part of a buffer that Read() must not touch
constexpr uint8_t CANARY = 0xa5;

template <typename IterT>
bool Untouched(const IterT begin, const IterT end) {
    return std::all_of(begin, end, [](const uint8_t byte) { return byte == CANARY; });
}

// ---------------------------------------------------------------------------------------------------------------------

// Driver connected to a TCP peer on an ephemeral localhost port, which plays the sensor
class TcpReadTest : public ::testing::Test {
   protected:
    void SetUp() override {
        listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        ASSERT_GE(listen_fd_, 0) << std::strerror(errno);
        struct sockaddr_in address;
        std::memset(&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        ASSERT_EQ(bind(listen_fd_, (struct sockaddr*)&address, sizeof(address)), 0) << std::strerror(errno);
        ASSERT_EQ(listen(listen_fd_, 1), 0) << std::strerror(errno);
        socklen_t address_size = sizeof(address);
        ASSERT_EQ(getsockname(listen_fd_, (struct sockaddr*)&address, &address_size), 0) << std::strerror(errno);

        DriverParams params;
        params.stream_ = "tcpcli://127.0.0.1:" + std::to_string(ntohs(address.sin_port));
        driver_ = std::make_unique<TestDriver>(params);
        ASSERT_TRUE(driver_->Connect());
        sensor_fd_ = accept(listen_fd_, nullptr, nullptr);
        ASSERT_GE(sensor_fd_, 0) << std::strerror(errno);
    }

    void TearDown() override {
        driver_.reset();
        if (sensor_fd_ >= 0) {
            close(sensor_fd_);
        }
        if (listen_fd_ >= 0) {
            close(listen_fd_);
        }
    }

    // TCP may accept fewer bytes than requested; no application framing is assumed.
    void SensorSends(const std::string& data) {
        std::size_t sent = 0;
        while (sent < data.size()) {
            const ssize_t size = send(sensor_fd_, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
            if ((size < 0) && (errno == EINTR)) {
                continue;
            }
            ASSERT_GT(size, 0) << std::strerror(errno);
            sent += static_cast<std::size_t>(size);
        }
    }

    void ReceiveBytes(const std::string& expected, const std::size_t request_size, std::size_t& largest_read) {
        std::string received;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(READ_TIMEOUT_MS);
        while (received.size() < expected.size()) {
            ASSERT_LT(std::chrono::steady_clock::now(), deadline) << "received " << received.size() << " bytes";
            std::array<uint8_t, 256> buf;
            buf.fill(CANARY);
            const std::size_t size = driver_->Read(buf.data(), request_size, 100);
            EXPECT_TRUE(Untouched(buf.begin() + request_size, buf.end()));
            ASSERT_LE(size, request_size);
            EXPECT_TRUE(Untouched(buf.begin() + size, buf.end()));
            ASSERT_TRUE(driver_->IsConnected());
            received.append(buf.begin(), buf.begin() + size);
            largest_read = std::max(largest_read, size);
        }
        EXPECT_EQ(received, expected);
    }

    std::unique_ptr<TestDriver> driver_;
    int listen_fd_ = -1;
    int sensor_fd_ = -1;
};

TEST_F(TcpReadTest, ReassemblesAnImuFrameAcrossShortReads) {
    // Hold back the rest to force a short read without a sleep or a packet-boundary assumption.
    constexpr std::size_t PREFIX_SIZE = 3;
    std::size_t largest_read = 0;
    ASSERT_NO_FATAL_FAILURE(SensorSends(CORRIMU_FRAME.substr(0, PREFIX_SIZE)));
    ASSERT_NO_FATAL_FAILURE(ReceiveBytes(CORRIMU_FRAME.substr(0, PREFIX_SIZE), 64, largest_read));
    ASSERT_NO_FATAL_FAILURE(SensorSends(CORRIMU_FRAME.substr(PREFIX_SIZE)));
    ASSERT_NO_FATAL_FAILURE(ReceiveBytes(CORRIMU_FRAME.substr(PREFIX_SIZE), 64, largest_read));
    RecordProperty("largest_tcp_read", static_cast<int>(largest_read));
}

TEST_F(TcpReadTest, NeverWritesPastTheRequestedSize) {
    ASSERT_NO_FATAL_FAILURE(SensorSends(CORRIMU_FRAME));

    // Fewer bytes than the 8 the old code read whatever the caller asked for
    constexpr std::size_t HEAD_SIZE = 5;
    std::size_t largest_read = 0;
    ASSERT_NO_FATAL_FAILURE(ReceiveBytes(CORRIMU_FRAME, HEAD_SIZE, largest_read));
}

TEST_F(TcpReadTest, ReconnectsUsingTheOriginalStreamSpec) {
    ASSERT_EQ(shutdown(sensor_fd_, SHUT_RDWR), 0);
    close(sensor_fd_);
    sensor_fd_ = -1;
    driver_->Disconnect();
    ASSERT_FALSE(driver_->IsConnected());
    ASSERT_TRUE(driver_->Connect());
    sensor_fd_ = accept(listen_fd_, nullptr, nullptr);
    ASSERT_GE(sensor_fd_, 0);
    ASSERT_NO_FATAL_FAILURE(SensorSends(CORRIMU_FRAME));
    std::size_t largest_read = 0;
    ASSERT_NO_FATAL_FAILURE(ReceiveBytes(CORRIMU_FRAME, 64, largest_read));
}

// ---------------------------------------------------------------------------------------------------------------------

// Driver connected to the slave side of a pseudo-terminal, which stands in for the sensor's serial port
class SerialReadTest : public ::testing::Test {
   protected:
    void SetUp() override {
        pty_fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        ASSERT_GE(pty_fd_, 0) << std::strerror(errno);
        ASSERT_EQ(grantpt(pty_fd_), 0) << std::strerror(errno);
        ASSERT_EQ(unlockpt(pty_fd_), 0) << std::strerror(errno);
        const char* port = ptsname(pty_fd_);
        ASSERT_NE(port, nullptr) << std::strerror(errno);

        DriverParams params;
        params.stream_ = std::string("serial://") + port + ":115200";
        driver_ = std::make_unique<TestDriver>(params);
        ASSERT_TRUE(driver_->Connect());
    }

    void TearDown() override {
        driver_.reset();
        if (pty_fd_ >= 0) {
            close(pty_fd_);
        }
    }

    std::unique_ptr<TestDriver> driver_;
    int pty_fd_ = -1;  // Master side, written by the "sensor"
};

TEST_F(SerialReadTest, ReadsTheFrameInRequestsOfTheCallersSize) {
    ASSERT_EQ(write(pty_fd_, CORRIMU_FRAME.data(), CORRIMU_FRAME.size()), (ssize_t)CORRIMU_FRAME.size())
        << std::strerror(errno);

    // The line discipline decides how many bytes each read() gets, so collect them. Every request is honoured.
    constexpr std::size_t REQUEST_SIZE = 16;
    std::string received;
    while (received.size() < CORRIMU_FRAME.size()) {
        std::array<uint8_t, 64> buf;
        buf.fill(CANARY);
        const std::size_t size = driver_->Read(buf.data(), REQUEST_SIZE, READ_TIMEOUT_MS);
        // recv() on a tty fails (ENOTSOCK), which dropped the connection
        ASSERT_TRUE(driver_->IsConnected());
        ASSERT_GT(size, 0u);
        ASSERT_LE(size, REQUEST_SIZE);
        EXPECT_TRUE(Untouched(buf.begin() + size, buf.end()));
        received.append(buf.begin(), buf.begin() + size);
    }
    EXPECT_EQ(received, CORRIMU_FRAME);
}

/* ****************************************************************************************************************** */
}  // namespace
