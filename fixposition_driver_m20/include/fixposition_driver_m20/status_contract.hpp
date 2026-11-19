/**
 * @file
 * @brief The wire contracts of drdds/LocationStatus — ROS-free
 *
 * The OEM has numbered the status fields twice, and which numbering a robot's navigation stack reads
 * follows from its OEM firmware. The contract is therefore a startup parameter (m20.status_contract),
 * and each one is a pure function from the monitor's StatusHealth to the values on the wire.
 */

#ifndef __FIXPOSITION_DRIVER_M20_STATUS_CONTRACT_HPP__
#define __FIXPOSITION_DRIVER_M20_STATUS_CONTRACT_HPP__

/* LIBC/STL */
#include <cstdint>
#include <string>

/* PACKAGE */
#include "fixposition_driver_m20/status_monitor.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief Which numbering /LOCATION_STATUS goes out in
 */
enum class StatusContract {
    LEGACY,  //!< "legacy": what the OEM handlers before firmware 120 read
    V1_2_0,  //!< "v1.2.0": the OEM's specification v1.2.0, localization baseline 3.5.13+
};

/**
 * @brief The values of the legacy contract
 *
 * What the field-validated Python bridge sent before the OEM had published any specification.
 */
namespace legacy {
constexpr uint8_t TOTAL_UNINITIALIZED = 0;  //!< needs a localization init / map
constexpr uint8_t TOTAL_NORMAL = 1;
constexpr uint8_t TOTAL_LOW_QUALITY = 2;  //!< degraded inputs or execution errors, keep watching
constexpr uint8_t TOTAL_LOST = 3;         //!< stop navigating and re-initialize
//! Every loss_val and exec_val flag
constexpr uint8_t ERR_OK = 0;
constexpr uint8_t ERR_ERROR = 1;
//! Every input_val field
constexpr uint8_t INPUT_OK = 1;
constexpr uint8_t INPUT_MISSING = 0;
}  // namespace legacy

/**
 * @brief The values of the OEM's "山猫M20 定位输出话题与格式说明 v1.2.0"
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
 * @brief The parameter value that names a contract
 */
const char* StatusContractName(StatusContract contract);

/**
 * @brief Read a contract from its parameter value
 *
 * @param[in]  text      the value of m20.status_contract
 * @param[out] contract  the contract it names; untouched unless it names one
 *
 * @returns false unless @p text is exactly "legacy" or "v1.2.0"
 */
bool ParseStatusContract(const std::string& text, StatusContract& contract);

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
 * @brief The legacy contract's values
 *
 * The contract has no "not used" input: an input that is not fresh is reported missing, whatever the
 * reason.
 */
StatusSnapshot EncodeLegacyStatus(const StatusHealth& health);

/**
 * @brief The v1.2.0 contract's values
 */
StatusSnapshot EncodeV120Status(const StatusHealth& health);

/**
 * @brief The values of @p contract
 */
StatusSnapshot EncodeStatus(StatusContract contract, const StatusHealth& health);

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_STATUS_CONTRACT_HPP__
