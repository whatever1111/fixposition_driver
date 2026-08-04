/**
 * @file
 * @brief Implementation of the localization health tracking (see status_monitor.hpp)
 */

/* LIBC/STL */
#include <limits>

/* PACKAGE */
#include "fixposition_driver_m20/status_monitor.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

namespace {
//! "Never happened" — any elapsed time against it exceeds any timeout
constexpr double NEVER = -std::numeric_limits<double>::infinity();
}  // namespace

StatusMonitor::StatusMonitor(const StatusTimeouts& timeouts, const bool matching_enabled)
    : timeouts_{timeouts}, matching_enabled_{matching_enabled}, last_odom_alive_{NEVER}, initialized_{false} {
    for (auto& entry : last_rx_) {
        entry.store(NEVER);
    }
    for (auto& entry : last_error_) {
        entry.store(NEVER);
    }
}

void StatusMonitor::MarkRx(const StatusInput input, const double now) {
    last_rx_[static_cast<std::size_t>(input)].store(now);
}

void StatusMonitor::MarkError(const StatusError error, const double now) {
    last_error_[static_cast<std::size_t>(error)].store(now);
}

void StatusMonitor::MarkOdomAlive(const double now) { last_odom_alive_.store(now); }

void StatusMonitor::SetInitialized() { initialized_.store(true); }

bool StatusMonitor::Initialized() const { return initialized_.load(); }

bool StatusMonitor::EverReceived(const StatusInput input) const {
    return last_rx_[static_cast<std::size_t>(input)].load() > NEVER;
}

double StatusMonitor::Timeout(const StatusInput input) const {
    switch (input) {
        case StatusInput::LIDAR:
            return timeouts_.lidar;
        case StatusInput::IMU:
            return timeouts_.imu;
        case StatusInput::LEG_ODOM:
            return timeouts_.leg_odom;
        case StatusInput::RTK:
            return timeouts_.rtk;
        case StatusInput::MATCHING:
            return timeouts_.matching;
        case StatusInput::COUNT:
            break;
    }
    return 0.0;
}

bool StatusMonitor::Fresh(const StatusInput input, const double now) const {
    const double last = last_rx_[static_cast<std::size_t>(input)].load();
    return (last > NEVER) && ((now - last) <= Timeout(input));
}

bool StatusMonitor::ErrorActive(const StatusError error, const double now) const {
    const double last = last_error_[static_cast<std::size_t>(error)].load();
    return (last > NEVER) && ((now - last) <= timeouts_.error_hold);
}

StatusSnapshot StatusMonitor::Compute(const double now) const {
    StatusSnapshot snapshot;

    // Inputs
    snapshot.lidar = Fresh(StatusInput::LIDAR, now) ? INPUT_OK : INPUT_MISSING;
    snapshot.imu = Fresh(StatusInput::IMU, now) ? INPUT_OK : INPUT_MISSING;
    snapshot.leg_odom = Fresh(StatusInput::LEG_ODOM, now) ? INPUT_OK : INPUT_MISSING;
    snapshot.rtk = Fresh(StatusInput::RTK, now) ? INPUT_OK : INPUT_MISSING;
    // Neither is fitted on this robot
    snapshot.tag = INPUT_MISSING;
    snapshot.reflective_strip = INPUT_MISSING;

    // Losses
    const double last_odom = last_odom_alive_.load();
    const bool odom_flowing = (last_odom > NEVER) && ((now - last_odom) <= timeouts_.rtk);
    snapshot.odom_error = odom_flowing ? ERR_OK : ERR_ERROR;
    snapshot.leg_odom_error = Fresh(StatusInput::LEG_ODOM, now) ? ERR_OK : ERR_ERROR;
    // Positioning-only mode does no scan matching, so there is nothing that could fail
    snapshot.matching_error =
        matching_enabled_ ? (Fresh(StatusInput::MATCHING, now) ? ERR_OK : ERR_ERROR) : ERR_OK;
    snapshot.pub_error = ErrorActive(StatusError::PUBLISH, now) ? ERR_ERROR : ERR_OK;

    // Execution
    snapshot.overtime = ErrorActive(StatusError::OVERTIME, now) ? ERR_ERROR : ERR_OK;
    snapshot.numeric_error = ErrorActive(StatusError::NUMERIC, now) ? ERR_ERROR : ERR_OK;
    snapshot.timestamp_error = ErrorActive(StatusError::TIMESTAMP, now) ? ERR_ERROR : ERR_OK;

    // Verdict
    if (!initialized_.load()) {
        snapshot.total_status = TOTAL_UNINITIALIZED;
    } else if (!odom_flowing || (snapshot.numeric_error == ERR_ERROR)) {
        snapshot.total_status = TOTAL_LOST;
    } else if ((snapshot.overtime == ERR_ERROR) || (snapshot.timestamp_error == ERR_ERROR) ||
               (snapshot.matching_error == ERR_ERROR) || (snapshot.pub_error == ERR_ERROR) ||
               (snapshot.lidar == INPUT_MISSING) || (snapshot.imu == INPUT_MISSING) ||
               (snapshot.rtk == INPUT_MISSING) || (snapshot.leg_odom == INPUT_MISSING)) {
        snapshot.total_status = TOTAL_LOW_QUALITY;
    } else {
        snapshot.total_status = TOTAL_NORMAL;
    }

    return snapshot;
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
