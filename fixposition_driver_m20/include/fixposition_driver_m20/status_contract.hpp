/**
 * @file
 * @brief The wire contract of drdds/LocationStatus — ROS-free
 *
 * /LOCATION_STATUS goes out in the numbering of the OEM's "山猫M20 定位输出话题与格式说明 v1.2.0",
 * which the navigation stack of OEM firmware 120 and later reads. The encoding is a pure function
 * from the monitor's StatusHealth to the values on the wire.
 */

#ifndef __FIXPOSITION_DRIVER_M20_STATUS_CONTRACT_HPP__
#define __FIXPOSITION_DRIVER_M20_STATUS_CONTRACT_HPP__

/* LIBC/STL */
#include <cstdint>

/* PACKAGE */
#include "fixposition_driver_m20/status_monitor.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief The values of the specification v1.2.0
 *
 * Downstream treats total_status 0 and 2 as usable. Some values the specification defines are never
 * sent: total_status 5 (lost, needs an initial pose), because the verdict has no such state — the
 * driver cannot tell whether the pipeline would recover without one; input 3 (abnormal), because
 * nothing here tells an abnormal input from a missing one; total_status 1 is undefined and 4 and 6
 * are reserved.
 */
namespace v1_2_0 {
constexpr uint8_t TOTAL_NORMAL = 0;
constexpr uint8_t TOTAL_MINOR = 2;         //!< usable, with a minor fault
constexpr uint8_t TOTAL_LOST = 3;          //!< lost, recovering by itself
constexpr uint8_t TOTAL_INITIALIZING = 7;  //!< not usable yet, including right after an initial pose
constexpr uint8_t INPUT_NOT_USED = 0;
constexpr uint8_t INPUT_NORMAL = 1;
constexpr uint8_t INPUT_NO_DATA = 2;
constexpr uint8_t LOSS_NORMAL = 0;
constexpr uint8_t LOSS_MINOR = 1;
constexpr uint8_t LOSS_HARD = 2;
constexpr uint8_t EXEC_OK = 0;
constexpr uint8_t EXEC_ERROR = 1;
}  // namespace v1_2_0

/**
 * @brief One status message's values, mapping 1:1 onto drdds/LocationStatusValue
 */
struct StatusSnapshot {
    uint8_t total_status = 0;
    // exec_val
    uint8_t overtime = 0;
    uint8_t numeric_error = 0;
    uint8_t timestamp_error = 0;
    // loss_val
    uint8_t odom_error = 0;
    uint8_t leg_odom_error = 0;
    uint8_t matching_error = 0;
    uint8_t pub_error = 0;
    // input_val
    uint8_t lidar = 0;
    uint8_t imu = 0;
    uint8_t leg_odom = 0;
    uint8_t rtk = 0;
    uint8_t tag = 0;
    uint8_t reflective_strip = 0;
};

/**
 * @brief The wire values of @p health
 *
 * While StatusHealth::initializing is set, total_status is 7 whatever the verdict; the other fields
 * are unaffected.
 */
StatusSnapshot EncodeStatus(const StatusHealth& health);

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_STATUS_CONTRACT_HPP__
