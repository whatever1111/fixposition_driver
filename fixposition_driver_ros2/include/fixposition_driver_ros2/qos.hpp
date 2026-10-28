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

#ifndef FIXPOSITION_DRIVER_ROS2_QOS_HPP_
#define FIXPOSITION_DRIVER_ROS2_QOS_HPP_

/* LIBC/STL */
#include <cstddef>
#include <string>

/* EXTERNAL */
#include "fixposition_driver_ros2/ext/rclcpp.hpp"

/* PACKAGE */

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief QoS of the output topics
 *
 * @param[in]  qos_type  The qos_type parameter: "sensor_short", "sensor_long", "default_short" or "default_long",
 *                       anything else selects the default-type QoS with a history of 10
 *
 * @returns the QoS
 */
rclcpp::QoS QosFromType(const std::string& qos_type);

/**
 * @brief QoS of the FP_A-RAWIMU and FP_A-CORRIMU output topics
 *
 * @param[in]  qos_type  The qos_type parameter, see QosFromType()
 * @param[in]  depth     History depth, see LoadImuQueueDepth()
 *
 * @returns the QoS of qos_type, with a history of depth messages
 */
rclcpp::QoS ImuQosFromType(const std::string& qos_type, const std::size_t depth);

/**
 * @brief Load the imu_queue_depth parameter, the history depth of the FP_A-RAWIMU and FP_A-CORRIMU output topics
 *
 * @param[in]  nh  Node handle
 *
 * @returns the history depth, 200 if the parameter is not set
 *
 * @throws std::invalid_argument if the parameter is out of range (1...2000), an rclcpp exception (a
 *         std::runtime_error) if it is not an integer
 */
std::size_t LoadImuQueueDepth(rclcpp::Node& nh);

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // FIXPOSITION_DRIVER_ROS2_QOS_HPP_
