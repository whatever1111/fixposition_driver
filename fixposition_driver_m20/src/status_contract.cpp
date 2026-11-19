/**
 * @file
 * @brief Implementation of the /LOCATION_STATUS wire contract (see status_contract.hpp)
 */

/* PACKAGE */
#include "fixposition_driver_m20/status_contract.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

namespace {

uint8_t Input(const InputState state) {
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

uint8_t Total(const StatusVerdict verdict) {
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

uint8_t Exec(const bool error) { return error ? v1_2_0::EXEC_ERROR : v1_2_0::EXEC_OK; }

}  // namespace

StatusSnapshot EncodeStatus(const StatusHealth& health) {
    StatusSnapshot snapshot;
    // The specification asks for "initializing" after an initial pose until the localization is stable
    snapshot.total_status = health.initializing ? v1_2_0::TOTAL_INITIALIZING : Total(health.verdict);
    snapshot.overtime = Exec(health.overtime);
    snapshot.numeric_error = Exec(health.numeric_error);
    snapshot.timestamp_error = Exec(health.timestamp_error);
    // Odometry that has stopped is the localization gone; every other loss is a degradation
    snapshot.odom_error = health.odom_stalled ? v1_2_0::LOSS_HARD : v1_2_0::LOSS_NORMAL;
    snapshot.leg_odom_error = health.leg_odom_error ? v1_2_0::LOSS_MINOR : v1_2_0::LOSS_NORMAL;
    snapshot.matching_error = health.matching_error ? v1_2_0::LOSS_MINOR : v1_2_0::LOSS_NORMAL;
    snapshot.pub_error = health.pub_error ? v1_2_0::LOSS_MINOR : v1_2_0::LOSS_NORMAL;
    snapshot.lidar = Input(health.lidar);
    snapshot.imu = Input(health.imu);
    snapshot.leg_odom = Input(health.leg_odom);
    snapshot.rtk = Input(health.rtk);
    snapshot.tag = Input(health.tag);
    snapshot.reflective_strip = Input(health.reflective_strip);
    return snapshot;
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
