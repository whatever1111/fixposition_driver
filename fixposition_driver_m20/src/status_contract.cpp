/**
 * @file
 * @brief Implementation of the /LOCATION_STATUS wire contracts (see status_contract.hpp)
 */

/* PACKAGE */
#include "fixposition_driver_m20/status_contract.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

namespace {

constexpr const char* LEGACY_NAME = "legacy";
constexpr const char* V1_2_0_NAME = "v1.2.0";

uint8_t LegacyFlag(const bool error) { return error ? legacy::ERR_ERROR : legacy::ERR_OK; }

//! The legacy contract has no "not used": an input that is not fresh is missing, whatever the reason
uint8_t LegacyInput(const InputState state) {
    return (state == InputState::FRESH) ? legacy::INPUT_OK : legacy::INPUT_MISSING;
}

uint8_t LegacyTotal(const StatusVerdict verdict) {
    switch (verdict) {
        case StatusVerdict::UNINITIALIZED:
            return legacy::TOTAL_UNINITIALIZED;
        case StatusVerdict::LOST:
            return legacy::TOTAL_LOST;
        case StatusVerdict::LOW_QUALITY:
            return legacy::TOTAL_LOW_QUALITY;
        case StatusVerdict::NORMAL:
            return legacy::TOTAL_NORMAL;
    }
    return legacy::TOTAL_LOST;
}

uint8_t V120Input(const InputState state) {
    switch (state) {
        case InputState::NOT_USED:
            return v1_2_0::INPUT_NOT_USED;
        case InputState::FRESH:
            return v1_2_0::INPUT_NORMAL;
        case InputState::MISSING:
            return v1_2_0::INPUT_NO_DATA;
    }
    return v1_2_0::INPUT_NO_DATA;
}

uint8_t V120Total(const StatusVerdict verdict) {
    switch (verdict) {
        case StatusVerdict::UNINITIALIZED:
            return v1_2_0::TOTAL_INITIALIZING;
        case StatusVerdict::LOST:
            return v1_2_0::TOTAL_LOST;
        case StatusVerdict::LOW_QUALITY:
            return v1_2_0::TOTAL_MINOR;
        case StatusVerdict::NORMAL:
            return v1_2_0::TOTAL_NORMAL;
    }
    return v1_2_0::TOTAL_LOST;
}

uint8_t V120Exec(const bool error) { return error ? v1_2_0::EXEC_ERROR : v1_2_0::EXEC_OK; }

}  // namespace

const char* StatusContractName(const StatusContract contract) {
    switch (contract) {
        case StatusContract::LEGACY:
            return LEGACY_NAME;
        case StatusContract::V1_2_0:
            return V1_2_0_NAME;
    }
    return LEGACY_NAME;
}

bool ParseStatusContract(const std::string& text, StatusContract& contract) {
    if (text == LEGACY_NAME) {
        contract = StatusContract::LEGACY;
        return true;
    }
    if (text == V1_2_0_NAME) {
        contract = StatusContract::V1_2_0;
        return true;
    }
    return false;
}

StatusSnapshot EncodeLegacyStatus(const StatusHealth& health) {
    StatusSnapshot snapshot;
    snapshot.total_status = LegacyTotal(health.verdict);
    snapshot.overtime = LegacyFlag(health.overtime);
    snapshot.numeric_error = LegacyFlag(health.numeric_error);
    snapshot.timestamp_error = LegacyFlag(health.timestamp_error);
    snapshot.odom_error = LegacyFlag(health.odom_stalled);
    snapshot.leg_odom_error = LegacyFlag(health.leg_odom_error);
    snapshot.matching_error = LegacyFlag(health.matching_error);
    snapshot.pub_error = LegacyFlag(health.pub_error);
    snapshot.lidar = LegacyInput(health.lidar);
    snapshot.imu = LegacyInput(health.imu);
    snapshot.leg_odom = LegacyInput(health.leg_odom);
    snapshot.rtk = LegacyInput(health.rtk);
    snapshot.tag = LegacyInput(health.tag);
    snapshot.reflective_strip = LegacyInput(health.reflective_strip);
    return snapshot;
}

StatusSnapshot EncodeV120Status(const StatusHealth& health) {
    StatusSnapshot snapshot;
    snapshot.total_status = V120Total(health.verdict);
    snapshot.overtime = V120Exec(health.overtime);
    snapshot.numeric_error = V120Exec(health.numeric_error);
    snapshot.timestamp_error = V120Exec(health.timestamp_error);
    // Odometry that has stopped is the localization gone; every other loss is a degradation
    snapshot.odom_error = health.odom_stalled ? v1_2_0::LOSS_HARD : v1_2_0::LOSS_NORMAL;
    snapshot.leg_odom_error = health.leg_odom_error ? v1_2_0::LOSS_MINOR : v1_2_0::LOSS_NORMAL;
    snapshot.matching_error = health.matching_error ? v1_2_0::LOSS_MINOR : v1_2_0::LOSS_NORMAL;
    snapshot.pub_error = health.pub_error ? v1_2_0::LOSS_MINOR : v1_2_0::LOSS_NORMAL;
    snapshot.lidar = V120Input(health.lidar);
    snapshot.imu = V120Input(health.imu);
    snapshot.leg_odom = V120Input(health.leg_odom);
    snapshot.rtk = V120Input(health.rtk);
    snapshot.tag = V120Input(health.tag);
    snapshot.reflective_strip = V120Input(health.reflective_strip);
    return snapshot;
}

StatusSnapshot EncodeStatus(const StatusContract contract, const StatusHealth& health) {
    switch (contract) {
        case StatusContract::LEGACY:
            return EncodeLegacyStatus(health);
        case StatusContract::V1_2_0:
            return EncodeV120Status(health);
    }
    return EncodeLegacyStatus(health);
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
