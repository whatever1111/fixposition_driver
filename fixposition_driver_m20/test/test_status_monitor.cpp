/**
 * @file
 * @brief Tests for the localization health state machine
 */

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include "fixposition_driver_m20/status_monitor.hpp"

namespace {

using fixposition::m20::ERR_ERROR;
using fixposition::m20::ERR_OK;
using fixposition::m20::INPUT_MISSING;
using fixposition::m20::INPUT_OK;
using fixposition::m20::StatusError;
using fixposition::m20::StatusInput;
using fixposition::m20::StatusMonitor;
using fixposition::m20::StatusTimeouts;
using fixposition::m20::TOTAL_LOST;
using fixposition::m20::TOTAL_LOW_QUALITY;
using fixposition::m20::TOTAL_NORMAL;
using fixposition::m20::TOTAL_UNINITIALIZED;

//! Mark every input and the odometry alive at @p now (the monitor itself is not copyable)
void MarkAllAlive(StatusMonitor& monitor, const double now, const bool with_matching = false) {
    monitor.MarkOdomAlive(now);
    monitor.MarkRx(StatusInput::LIDAR, now);
    monitor.MarkRx(StatusInput::IMU, now);
    monitor.MarkRx(StatusInput::LEG_ODOM, now);
    monitor.MarkRx(StatusInput::RTK, now);
    if (with_matching) {
        monitor.MarkRx(StatusInput::MATCHING, now);
    }
}

//! Bring a monitor to "everything alive and initialized at t = 1.0"
void MakeHealthy(StatusMonitor& monitor, const bool with_matching = false) {
    monitor.SetInitialized();
    MarkAllAlive(monitor, 1.0, with_matching);
}

}  // namespace

TEST(StatusMonitorTest, StartsUninitialized) {
    const StatusMonitor monitor{StatusTimeouts(), false};
    const auto snapshot = monitor.Compute(0.0);
    EXPECT_EQ(snapshot.total_status, TOTAL_UNINITIALIZED);
    EXPECT_EQ(snapshot.lidar, INPUT_MISSING);
    EXPECT_EQ(snapshot.odom_error, ERR_ERROR);
    EXPECT_FALSE(monitor.EverReceived(StatusInput::LIDAR));
}

TEST(StatusMonitorTest, ReportsNormalWhenEverythingIsAlive) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    const auto snapshot = monitor.Compute(1.05);
    EXPECT_EQ(snapshot.total_status, TOTAL_NORMAL);
    EXPECT_EQ(snapshot.lidar, INPUT_OK);
    EXPECT_EQ(snapshot.imu, INPUT_OK);
    EXPECT_EQ(snapshot.leg_odom, INPUT_OK);
    EXPECT_EQ(snapshot.rtk, INPUT_OK);
    EXPECT_EQ(snapshot.odom_error, ERR_OK);
    EXPECT_EQ(snapshot.matching_error, ERR_OK);
    // Neither is fitted on this robot, so they stay missing even when everything is well
    EXPECT_EQ(snapshot.tag, INPUT_MISSING);
    EXPECT_EQ(snapshot.reflective_strip, INPUT_MISSING);
}

TEST(StatusMonitorTest, MissingLidarIsLowQuality) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    // Past the lidar timeout (0.5 s) but within every other one
    const auto snapshot = monitor.Compute(1.6);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOW_QUALITY);
    EXPECT_EQ(snapshot.lidar, INPUT_MISSING);
    EXPECT_EQ(snapshot.rtk, INPUT_OK);
}

TEST(StatusMonitorTest, StalledOdometryIsLost) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    const auto snapshot = monitor.Compute(3.0);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOST);
    EXPECT_EQ(snapshot.odom_error, ERR_ERROR);
}

TEST(StatusMonitorTest, NumericErrorIsLost) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    monitor.MarkError(StatusError::NUMERIC, 1.0);
    const auto snapshot = monitor.Compute(1.05);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOST);
    EXPECT_EQ(snapshot.numeric_error, ERR_ERROR);
}

TEST(StatusMonitorTest, OvertimeIsLowQualityAndDecaysAway) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    monitor.MarkError(StatusError::OVERTIME, 1.0);

    const auto during = monitor.Compute(1.05);
    EXPECT_EQ(during.total_status, TOTAL_LOW_QUALITY);
    EXPECT_EQ(during.overtime, ERR_ERROR);

    // Keep the inputs alive past the 5 s error hold and the flag clears
    MarkAllAlive(monitor, 7.0);
    const auto after = monitor.Compute(7.05);
    EXPECT_EQ(after.overtime, ERR_OK);
    EXPECT_EQ(after.total_status, TOTAL_NORMAL);
}

TEST(StatusMonitorTest, MatchingIsOnlyCheckedWhenEnabled) {
    // Positioning-only mode: nothing does scan matching, so its absence is not a fault
    StatusMonitor without{StatusTimeouts(), false};
    MakeHealthy(without);
    EXPECT_EQ(without.Compute(1.05).matching_error, ERR_OK);
    EXPECT_EQ(without.Compute(1.05).total_status, TOTAL_NORMAL);

    // SLAM mode: matching odometry that stops arriving degrades the status
    StatusMonitor stale{StatusTimeouts(), true};
    MakeHealthy(stale, true);
    EXPECT_EQ(stale.Compute(1.05).total_status, TOTAL_NORMAL);
    MarkAllAlive(stale, 2.0);  // everything but the matching odometry keeps flowing
    const auto snapshot = stale.Compute(2.05);  // matching last seen at 1.0, timeout 0.5
    EXPECT_EQ(snapshot.matching_error, ERR_ERROR);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOW_QUALITY);
}

TEST(StatusMonitorTest, PublishFailureIsLowQuality) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    monitor.MarkError(StatusError::PUBLISH, 1.0);
    const auto snapshot = monitor.Compute(1.05);
    EXPECT_EQ(snapshot.pub_error, ERR_ERROR);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOW_QUALITY);
}

TEST(StatusMonitorTest, EverReceivedIgnoresTimeouts) {
    StatusMonitor monitor{StatusTimeouts(), false};
    monitor.MarkRx(StatusInput::IMU, 1.0);
    EXPECT_TRUE(monitor.EverReceived(StatusInput::IMU));
    EXPECT_FALSE(monitor.Fresh(StatusInput::IMU, 100.0));  // long stale, but it was heard once
    EXPECT_FALSE(monitor.EverReceived(StatusInput::LIDAR));
}

TEST(StatusMonitorTest, HonoursCustomTimeouts) {
    StatusTimeouts timeouts;
    timeouts.lidar = 2.0;
    StatusMonitor monitor{timeouts, false};
    monitor.MarkRx(StatusInput::LIDAR, 1.0);
    EXPECT_TRUE(monitor.Fresh(StatusInput::LIDAR, 2.5));
    EXPECT_FALSE(monitor.Fresh(StatusInput::LIDAR, 3.5));
}
