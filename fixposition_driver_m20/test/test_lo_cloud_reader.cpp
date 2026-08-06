/**
 *  @file
 *  @brief Tests for the loopback-only cloud reader
 *
 * The full data path (OEM writer -> reader) needs the robot; here we cover the topic-name mangling
 * and that a participant, topic and reader actually come up and tear down cleanly on an isolated
 * domain.
 */

/* LIBC/STL */
#include <atomic>
#include <memory>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include "fixposition_driver_m20/lo_cloud_reader.hpp"

namespace {

using fixposition::m20::LoCloudReader;

// Well away from any deployment (deployments run domain 0)
constexpr uint32_t kTestDomain = 89;

TEST(LoCloudReaderTest, DdsTopicNameMangling) {
    EXPECT_EQ(LoCloudReader::DdsTopicName("/LIDAR/POINTS"), "rt/LIDAR/POINTS");
    EXPECT_EQ(LoCloudReader::DdsTopicName("/a"), "rt/a");
    EXPECT_EQ(LoCloudReader::DdsTopicName("relative"), "rt/relative");
    EXPECT_EQ(LoCloudReader::DdsTopicName(""), "rt/");
}

TEST(LoCloudReaderTest, ConstructsAndDestructs) {
    std::atomic<int> received{0};
    {
        LoCloudReader reader("/LIDAR/POINTS", kTestDomain, 5,
                             [&received](sensor_msgs::msg::PointCloud2::UniquePtr) { received++; });
        EXPECT_TRUE(reader.ok());
    }
    // Nothing publishes on the test domain; the callback must never have fired
    EXPECT_EQ(received.load(), 0);
}

TEST(LoCloudReaderTest, TwoReadersCoexist) {
    LoCloudReader a("/LIDAR/POINTS", kTestDomain, 5, [](sensor_msgs::msg::PointCloud2::UniquePtr) {});
    LoCloudReader b("/LIDAR/POINTS2", kTestDomain, 5, [](sensor_msgs::msg::PointCloud2::UniquePtr) {});
    EXPECT_TRUE(a.ok());
    EXPECT_TRUE(b.ok());
}

}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
