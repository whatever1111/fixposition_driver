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

InputState StatusMonitor::Monitored(const StatusInput input, const double now) const {
    return Fresh(input, now) ? InputState::FRESH : InputState::MISSING;
}

StatusHealth StatusMonitor::Compute(const double now) const {
    StatusHealth health;

    // Inputs
    health.lidar = Monitored(StatusInput::LIDAR, now);
    health.imu = Monitored(StatusInput::IMU, now);
    health.leg_odom = Monitored(StatusInput::LEG_ODOM, now);
    health.rtk = Monitored(StatusInput::RTK, now);
    // Neither is fitted on this robot
    health.tag = InputState::NOT_USED;
    health.reflective_strip = InputState::NOT_USED;

    // Losses
    const double last_odom = last_odom_alive_.load();
    const bool odom_flowing = (last_odom > NEVER) && ((now - last_odom) <= timeouts_.rtk);
    health.odom_stalled = !odom_flowing;
    health.leg_odom_error = !Fresh(StatusInput::LEG_ODOM, now);
    // Positioning-only mode does no scan matching, so there is nothing that could fail
    health.matching_error = matching_enabled_ && !Fresh(StatusInput::MATCHING, now);
    health.pub_error = ErrorActive(StatusError::PUBLISH, now);

    // Execution
    health.overtime = ErrorActive(StatusError::OVERTIME, now);
    health.numeric_error = ErrorActive(StatusError::NUMERIC, now);
    health.timestamp_error = ErrorActive(StatusError::TIMESTAMP, now);

    // Verdict. Only a missing input degrades it; one this configuration does not use never does.
    const auto missing = [](const InputState state) { return state == InputState::MISSING; };
    if (!initialized_.load()) {
        health.verdict = StatusVerdict::UNINITIALIZED;
    } else if (!odom_flowing || health.numeric_error) {
        health.verdict = StatusVerdict::LOST;
    } else if (health.overtime || health.timestamp_error || health.matching_error || health.pub_error ||
               missing(health.lidar) || missing(health.imu) || missing(health.rtk) || missing(health.leg_odom)) {
        health.verdict = StatusVerdict::LOW_QUALITY;
    } else {
        health.verdict = StatusVerdict::NORMAL;
    }

    return health;
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
