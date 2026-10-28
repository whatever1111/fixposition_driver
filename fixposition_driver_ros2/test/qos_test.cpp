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
 * @brief Fixposition driver node for ROS2: tests for the QoS of the output topics
 */

/* LIBC/STL */
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

#include "fixposition_driver_ros2/ext/rclcpp.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wshadow"
#include <fixposition_driver_msgs/msg/fpa_imu.hpp>
#pragma GCC diagnostic pop

/* PACKAGE */
#include "fixposition_driver_ros2/qos.hpp"

namespace {
/* ****************************************************************************************************************** */

using fixposition::ImuQosFromType;
using fixposition::LoadImuQueueDepth;
using fixposition::QosFromType;
using FpaImu = fixposition_driver_msgs::msg::FpaImu;

// The driver's node, with parameters as a launch or parameter file sets them
std::shared_ptr<rclcpp::Node> DriverNode(const std::vector<rclcpp::Parameter>& parameters = {}) {
    return std::make_shared<rclcpp::Node>("fixposition_driver", rclcpp::NodeOptions().parameter_overrides(parameters));
}

// The QoS that the DDS writer of a publisher actually got
rmw_qos_profile_t ActualQos(const rclcpp::PublisherBase& pub) { return pub.get_actual_qos().get_rmw_qos_profile(); }

// ---------------------------------------------------------------------------------------------------------------------

TEST(LoadImuQueueDepthTest, DefaultsTo200) {
    auto nh = DriverNode();
    EXPECT_EQ(LoadImuQueueDepth(*nh), 200u);
}

TEST(LoadImuQueueDepthTest, AcceptsDepthsFrom1To2000) {
    auto nh_min = DriverNode({rclcpp::Parameter("imu_queue_depth", 1)});
    EXPECT_EQ(LoadImuQueueDepth(*nh_min), 1u);
    auto nh_max = DriverNode({rclcpp::Parameter("imu_queue_depth", 2000)});
    EXPECT_EQ(LoadImuQueueDepth(*nh_max), 2000u);
}

TEST(LoadImuQueueDepthTest, RejectsDepthsOutOfRange) {
    for (const int64_t depth :
         {int64_t{0}, int64_t{-1}, int64_t{2001}, (int64_t{1} << 32) + 200, std::numeric_limits<int64_t>::max()}) {
        auto nh = DriverNode({rclcpp::Parameter("imu_queue_depth", depth)});
        EXPECT_THROW(LoadImuQueueDepth(*nh), std::invalid_argument) << "imu_queue_depth=" << depth;
    }
}

TEST(LoadImuQueueDepthTest, RejectsNonIntegers) {
    // E.g. "imu_queue_depth: 200.0" or "imu_queue_depth: '200'" in a parameter file
    auto nh_double = DriverNode({rclcpp::Parameter("imu_queue_depth", 200.0)});
    EXPECT_THROW(LoadImuQueueDepth(*nh_double), std::runtime_error);
    auto nh_string = DriverNode({rclcpp::Parameter("imu_queue_depth", std::string("200"))});
    EXPECT_THROW(LoadImuQueueDepth(*nh_string), std::runtime_error);
}

// ---------------------------------------------------------------------------------------------------------------------

// Output topic QoS per qos_type, as the driver has always had them
struct QosTypeQos {
    std::string qos_type;
    rmw_qos_reliability_policy_t reliability;
    std::size_t depth;
};
const std::vector<QosTypeQos> QOS_TYPES = {
    {"sensor_short", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, 5},  // M20
    {"sensor_long", RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, 10},
    {"default_short", RMW_QOS_POLICY_RELIABILITY_RELIABLE, 5},
    {"default_long", RMW_QOS_POLICY_RELIABILITY_RELIABLE, 10},
    {"unknown", RMW_QOS_POLICY_RELIABILITY_RELIABLE, 10},
};

TEST(QosTest, OnlyTheImuTopicsGetTheImuQueueDepth) {
    for (const auto& expected : QOS_TYPES) {
        SCOPED_TRACE("qos_type=" + expected.qos_type);

        // Publishers made the way the node makes them, for the IMU topics with the default imu_queue_depth
        auto nh = DriverNode();
        auto pub = nh->create_publisher<FpaImu>("/fixposition/fpa/other", QosFromType(expected.qos_type));
        auto imu_pub = nh->create_publisher<FpaImu>("/fixposition/fpa/rawimu",
                                                    ImuQosFromType(expected.qos_type, LoadImuQueueDepth(*nh)));
        const rmw_qos_profile_t qos = ActualQos(*pub);
        const rmw_qos_profile_t imu_qos = ActualQos(*imu_pub);

        // The other topics are as before
        EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        EXPECT_EQ(qos.depth, expected.depth);
        EXPECT_EQ(qos.reliability, expected.reliability);
        EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

        // The IMU topics only keep a longer history
        EXPECT_EQ(imu_qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        EXPECT_EQ(imu_qos.depth, 200u);
        EXPECT_EQ(imu_qos.reliability, expected.reliability);
        EXPECT_EQ(imu_qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    }
}

TEST(QosTest, TheLargestImuQueueDepthIsAccepted) {
    auto nh = DriverNode({rclcpp::Parameter("imu_queue_depth", 2000)});
    auto imu_pub =
        nh->create_publisher<FpaImu>("/fixposition/fpa/rawimu", ImuQosFromType("sensor_short", LoadImuQueueDepth(*nh)));
    EXPECT_EQ(ActualQos(*imu_pub).depth, 2000u);
}

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
