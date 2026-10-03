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
 * @brief QoS of the output topics
 */

/* LIBC/STL */
#include <cstdint>
#include <stdexcept>

/* EXTERNAL */

/* PACKAGE */
#include "fixposition_driver_ros2/qos.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

static constexpr const char* IMU_QUEUE_DEPTH = "imu_queue_depth";

// 1 s of FP_A-RAWIMU/CORRIMU at 200 Hz. ROS 2 Foxy (rmw_fastrtps) publishes asynchronously, which makes the writer
// history the send queue: with the 5 or 10 of the qos_type, a send stall of more than 25 or 50 ms dropped messages.
static constexpr int64_t IMU_QUEUE_DEPTH_DEFAULT = 200;

// 10 s at 200 Hz: rejects typos (e.g. an extra zero) rather than limiting any sensible configuration
static constexpr int64_t IMU_QUEUE_DEPTH_MAX = 2000;

rclcpp::QoS QosFromType(const std::string& qos_type) {
    rclcpp::QoS qos(rclcpp::KeepLast(10), rmw_qos_profile_default);
    // - Short-queue sensor-type QoS
    if (qos_type == "sensor_short") {
        qos = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data);
    }
    // - Long-queue sensor-type QoS
    else if (qos_type == "sensor_long") {
        qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);
    }
    // - Short-queue default-type QoS
    else if (qos_type == "default_short") {
        qos = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_default);
    }
    // - Long-queue default-type QoS
    else if (qos_type == "default_long") {
        qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_default);
    }
    return qos;
}

rclcpp::QoS ImuQosFromType(const std::string& qos_type, const std::size_t depth) {
    rclcpp::QoS qos = QosFromType(qos_type);
    qos.keep_last(depth);
    return qos;
}

std::size_t LoadImuQueueDepth(rclcpp::Node& nh) {
    // The integer default makes this an integer parameter: other types throw (rclcpp::ParameterTypeException on Foxy,
    // rclcpp::exceptions::InvalidParameterTypeException on later versions)
    const int64_t depth = nh.declare_parameter<int64_t>(IMU_QUEUE_DEPTH, IMU_QUEUE_DEPTH_DEFAULT);
    if ((depth < 1) || (depth > IMU_QUEUE_DEPTH_MAX)) {
        throw std::invalid_argument(std::string("Bad value for ") + IMU_QUEUE_DEPTH + " param: " +
                                    std::to_string(depth) + ", must be 1..." + std::to_string(IMU_QUEUE_DEPTH_MAX));
    }
    return static_cast<std::size_t>(depth);
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
