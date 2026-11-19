/**
 * @file
 * @brief Tests for the localization health state machine and the wire contracts it is encoded in
 *
 * The contract tables below are written out in numbers, as the two specifications give them.
 */

/* LIBC/STL */
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include "fixposition_driver_m20/status_contract.hpp"
#include "fixposition_driver_m20/status_monitor.hpp"

namespace {

using fixposition::m20::EncodeLegacyStatus;
using fixposition::m20::EncodeStatus;
using fixposition::m20::EncodeV120Status;
using fixposition::m20::InputState;
using fixposition::m20::ParseStatusContract;
using fixposition::m20::StatusContract;
using fixposition::m20::StatusContractName;
using fixposition::m20::StatusError;
using fixposition::m20::StatusHealth;
using fixposition::m20::StatusInput;
using fixposition::m20::StatusMonitor;
using fixposition::m20::StatusSnapshot;
using fixposition::m20::StatusTimeouts;
using fixposition::m20::StatusVerdict;
using fixposition::m20::legacy::ERR_ERROR;
using fixposition::m20::legacy::ERR_OK;
using fixposition::m20::legacy::INPUT_MISSING;
using fixposition::m20::legacy::INPUT_OK;
using fixposition::m20::legacy::TOTAL_LOST;
using fixposition::m20::legacy::TOTAL_LOW_QUALITY;
using fixposition::m20::legacy::TOTAL_NORMAL;
using fixposition::m20::legacy::TOTAL_UNINITIALIZED;

//! What the monitor reports at @p now, in the legacy contract
StatusSnapshot Legacy(const StatusMonitor& monitor, const double now) {
    return EncodeLegacyStatus(monitor.Compute(now));
}

//! What the monitor reports at @p now, in the v1.2.0 contract
StatusSnapshot V120(const StatusMonitor& monitor, const double now) { return EncodeV120Status(monitor.Compute(now)); }

//! A healthy evaluation: everything monitored is fresh, nothing is held
StatusHealth HealthyHealth() {
    StatusHealth health;
    health.verdict = StatusVerdict::NORMAL;
    health.odom_stalled = false;
    health.leg_odom_error = false;
    health.lidar = InputState::FRESH;
    health.imu = InputState::FRESH;
    health.leg_odom = InputState::FRESH;
    health.rtk = InputState::FRESH;
    return health;
}

//! (total_status, the three exec_val flags), as ints so that a failure prints numbers
using TotalAndExec = std::tuple<int, int, int, int>;
TotalAndExec TotalAndExecOf(const StatusSnapshot& snapshot) {
    return TotalAndExec{snapshot.total_status, snapshot.overtime, snapshot.numeric_error, snapshot.timestamp_error};
}

//! The four loss_val fields
using Losses = std::tuple<int, int, int, int>;
Losses LossesOf(const StatusSnapshot& snapshot) {
    return Losses{snapshot.odom_error, snapshot.leg_odom_error, snapshot.matching_error, snapshot.pub_error};
}

//! The six input_val fields
using Inputs = std::tuple<int, int, int, int, int, int>;
Inputs InputsOf(const StatusSnapshot& snapshot) {
    return Inputs{snapshot.lidar, snapshot.imu, snapshot.leg_odom,
                  snapshot.rtk,   snapshot.tag, snapshot.reflective_strip};
}

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
    const auto snapshot = Legacy(monitor, 0.0);
    EXPECT_EQ(snapshot.total_status, TOTAL_UNINITIALIZED);
    EXPECT_EQ(snapshot.lidar, INPUT_MISSING);
    EXPECT_EQ(snapshot.odom_error, ERR_ERROR);
    EXPECT_FALSE(monitor.EverReceived(StatusInput::LIDAR));
}

TEST(StatusMonitorTest, ReportsNormalWhenEverythingIsAlive) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    const auto snapshot = Legacy(monitor, 1.05);
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
    const auto snapshot = Legacy(monitor, 1.6);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOW_QUALITY);
    EXPECT_EQ(snapshot.lidar, INPUT_MISSING);
    EXPECT_EQ(snapshot.rtk, INPUT_OK);
}

TEST(StatusMonitorTest, StalledOdometryIsLost) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    const auto snapshot = Legacy(monitor, 3.0);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOST);
    EXPECT_EQ(snapshot.odom_error, ERR_ERROR);
}

TEST(StatusMonitorTest, NumericErrorIsLost) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    monitor.MarkError(StatusError::NUMERIC, 1.0);
    const auto snapshot = Legacy(monitor, 1.05);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOST);
    EXPECT_EQ(snapshot.numeric_error, ERR_ERROR);
}

TEST(StatusMonitorTest, OvertimeIsLowQualityAndDecaysAway) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    monitor.MarkError(StatusError::OVERTIME, 1.0);

    const auto during = Legacy(monitor, 1.05);
    EXPECT_EQ(during.total_status, TOTAL_LOW_QUALITY);
    EXPECT_EQ(during.overtime, ERR_ERROR);

    // Keep the inputs alive past the 5 s error hold and the flag clears
    MarkAllAlive(monitor, 7.0);
    const auto after = Legacy(monitor, 7.05);
    EXPECT_EQ(after.overtime, ERR_OK);
    EXPECT_EQ(after.total_status, TOTAL_NORMAL);
}

TEST(StatusMonitorTest, MatchingIsOnlyCheckedWhenEnabled) {
    // Positioning-only mode: nothing does scan matching, so its absence is not a fault
    StatusMonitor without{StatusTimeouts(), false};
    MakeHealthy(without);
    EXPECT_EQ(Legacy(without, 1.05).matching_error, ERR_OK);
    EXPECT_EQ(Legacy(without, 1.05).total_status, TOTAL_NORMAL);

    // SLAM mode: matching odometry that stops arriving degrades the status
    StatusMonitor stale{StatusTimeouts(), true};
    MakeHealthy(stale, true);
    EXPECT_EQ(Legacy(stale, 1.05).total_status, TOTAL_NORMAL);
    MarkAllAlive(stale, 2.0);  // everything but the matching odometry keeps flowing
    const auto snapshot = Legacy(stale, 2.05);  // matching last seen at 1.0, timeout 0.5
    EXPECT_EQ(snapshot.matching_error, ERR_ERROR);
    EXPECT_EQ(snapshot.total_status, TOTAL_LOW_QUALITY);
}

TEST(StatusMonitorTest, PublishFailureIsLowQuality) {
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    monitor.MarkError(StatusError::PUBLISH, 1.0);
    const auto snapshot = Legacy(monitor, 1.05);
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

// ---------------------------------------------------------------------------------------------------------------------

namespace {

//! A loss_val or exec_val flag and what each contract sends while it is raised
struct FlagField {
    const char* name;
    bool StatusHealth::* health;
    uint8_t StatusSnapshot::* wire;
    int legacy;
    int v120;
};

const FlagField FLAG_FIELDS[] = {
    {"exec_val.overtime", &StatusHealth::overtime, &StatusSnapshot::overtime, 1, 1},
    {"exec_val.numeric_error", &StatusHealth::numeric_error, &StatusSnapshot::numeric_error, 1, 1},
    {"exec_val.timestamp_error", &StatusHealth::timestamp_error, &StatusSnapshot::timestamp_error, 1, 1},
    {"loss_val.odom_error", &StatusHealth::odom_stalled, &StatusSnapshot::odom_error, 1, 2},
    {"loss_val.leg_odom_error", &StatusHealth::leg_odom_error, &StatusSnapshot::leg_odom_error, 1, 1},
    {"loss_val.matching_error", &StatusHealth::matching_error, &StatusSnapshot::matching_error, 1, 1},
    {"loss_val.pub_error", &StatusHealth::pub_error, &StatusSnapshot::pub_error, 1, 1},
};

//! An input_val field
struct InputField {
    const char* name;
    InputState StatusHealth::* health;
    uint8_t StatusSnapshot::* wire;
};

const InputField INPUT_FIELDS[] = {
    {"input_val.lidar", &StatusHealth::lidar, &StatusSnapshot::lidar},
    {"input_val.imu", &StatusHealth::imu, &StatusSnapshot::imu},
    {"input_val.leg_odom", &StatusHealth::leg_odom, &StatusSnapshot::leg_odom},
    {"input_val.rtk", &StatusHealth::rtk, &StatusSnapshot::rtk},
    {"input_val.tag", &StatusHealth::tag, &StatusSnapshot::tag},
    {"input_val.reflective_strip", &StatusHealth::reflective_strip, &StatusSnapshot::reflective_strip},
};

//! Every field of a snapshot but @p skip, as ints
std::vector<int> FieldsExcept(const StatusSnapshot& snapshot, uint8_t StatusSnapshot::* skip) {
    std::vector<int> values;
    for (uint8_t StatusSnapshot::* field :
         {&StatusSnapshot::total_status, &StatusSnapshot::overtime, &StatusSnapshot::numeric_error,
          &StatusSnapshot::timestamp_error, &StatusSnapshot::odom_error, &StatusSnapshot::leg_odom_error,
          &StatusSnapshot::matching_error, &StatusSnapshot::pub_error, &StatusSnapshot::lidar, &StatusSnapshot::imu,
          &StatusSnapshot::leg_odom, &StatusSnapshot::rtk, &StatusSnapshot::tag, &StatusSnapshot::reflective_strip}) {
        if (field != skip) {
            values.push_back(snapshot.*field);
        }
    }
    return values;
}

}  // namespace

TEST(StatusContractTest, ParsesExactlyTheTwoNames) {
    StatusContract contract = StatusContract::LEGACY;
    EXPECT_TRUE(ParseStatusContract("v1.2.0", contract));
    EXPECT_EQ(contract, StatusContract::V1_2_0);
    EXPECT_TRUE(ParseStatusContract("legacy", contract));
    EXPECT_EQ(contract, StatusContract::LEGACY);

    // Anything else is refused and leaves the value alone
    for (const char* text : {"", "v1.2", "1.2.0", "V1.2.0", "v1.2.0 ", " legacy", "Legacy", "v1.3.0", "v120"}) {
        StatusContract untouched = StatusContract::V1_2_0;
        EXPECT_FALSE(ParseStatusContract(text, untouched)) << '"' << text << '"';
        EXPECT_EQ(untouched, StatusContract::V1_2_0) << '"' << text << '"';
    }

    // The name of each contract, which the module declares as the default and logs, reads back as it
    for (const StatusContract each : {StatusContract::LEGACY, StatusContract::V1_2_0}) {
        StatusContract parsed = (each == StatusContract::LEGACY) ? StatusContract::V1_2_0 : StatusContract::LEGACY;
        EXPECT_TRUE(ParseStatusContract(StatusContractName(each), parsed));
        EXPECT_EQ(parsed, each);
    }
}

TEST(StatusContractTest, LegacyTable) {
    // total_status: 0 uninitialized, 1 normal, 2 low quality, 3 lost
    const std::vector<std::pair<StatusVerdict, int>> totals = {{StatusVerdict::UNINITIALIZED, 0},
                                                               {StatusVerdict::NORMAL, 1},
                                                               {StatusVerdict::LOW_QUALITY, 2},
                                                               {StatusVerdict::LOST, 3}};
    for (const auto& [verdict, expected] : totals) {
        StatusHealth health = HealthyHealth();
        health.verdict = verdict;
        EXPECT_EQ(EncodeLegacyStatus(health).total_status, expected) << static_cast<int>(verdict);
    }

    // Healthy: every input 1 but the two that are not fitted, every flag 0
    const StatusSnapshot healthy = EncodeLegacyStatus(HealthyHealth());
    EXPECT_EQ(TotalAndExecOf(healthy), TotalAndExec(1, 0, 0, 0));
    EXPECT_EQ(LossesOf(healthy), Losses(0, 0, 0, 0));
    EXPECT_EQ(InputsOf(healthy), Inputs(1, 1, 1, 1, 0, 0));

    // Each flag is 1 while raised, and touches no other field
    for (const FlagField& flag : FLAG_FIELDS) {
        StatusHealth health = HealthyHealth();
        health.*flag.health = true;
        const StatusSnapshot snapshot = EncodeLegacyStatus(health);
        EXPECT_EQ(snapshot.*flag.wire, flag.legacy) << flag.name;
        EXPECT_EQ(FieldsExcept(snapshot, flag.wire), FieldsExcept(healthy, flag.wire)) << flag.name;
    }

    // Each input is 1 while fresh and 0 otherwise, whether missing or not used at all
    const std::vector<std::pair<InputState, int>> states = {
        {InputState::FRESH, 1}, {InputState::MISSING, 0}, {InputState::NOT_USED, 0}};
    for (const InputField& input : INPUT_FIELDS) {
        for (const auto& [state, expected] : states) {
            StatusHealth health = HealthyHealth();
            health.*input.health = state;
            const StatusSnapshot snapshot = EncodeLegacyStatus(health);
            EXPECT_EQ(snapshot.*input.wire, expected) << input.name << " " << static_cast<int>(state);
            EXPECT_EQ(FieldsExcept(snapshot, input.wire), FieldsExcept(healthy, input.wire)) << input.name;
        }
    }
}

TEST(StatusContractTest, V120Table) {
    // total_status: 0 normal and 2 minor fault are usable; 3 lost and 7 initializing are not
    const std::vector<std::pair<StatusVerdict, int>> totals = {{StatusVerdict::UNINITIALIZED, 7},
                                                               {StatusVerdict::NORMAL, 0},
                                                               {StatusVerdict::LOW_QUALITY, 2},
                                                               {StatusVerdict::LOST, 3}};
    for (const auto& [verdict, expected] : totals) {
        StatusHealth health = HealthyHealth();
        health.verdict = verdict;
        EXPECT_EQ(EncodeV120Status(health).total_status, expected) << static_cast<int>(verdict);
    }

    // Healthy: every monitored input 1 (normal), the two not fitted 0 (not used), everything else 0
    const StatusSnapshot healthy = EncodeV120Status(HealthyHealth());
    EXPECT_EQ(TotalAndExecOf(healthy), TotalAndExec(0, 0, 0, 0));
    EXPECT_EQ(LossesOf(healthy), Losses(0, 0, 0, 0));
    EXPECT_EQ(InputsOf(healthy), Inputs(1, 1, 1, 1, 0, 0));

    // Execution flags are 0 / 1; stopped odometry is a hard loss (2), every other loss a minor one (1)
    for (const FlagField& flag : FLAG_FIELDS) {
        StatusHealth health = HealthyHealth();
        health.*flag.health = true;
        const StatusSnapshot snapshot = EncodeV120Status(health);
        EXPECT_EQ(snapshot.*flag.wire, flag.v120) << flag.name;
        EXPECT_EQ(FieldsExcept(snapshot, flag.wire), FieldsExcept(healthy, flag.wire)) << flag.name;
    }

    // Inputs: 0 not used, 1 normal, 2 no data
    const std::vector<std::pair<InputState, int>> states = {
        {InputState::NOT_USED, 0}, {InputState::FRESH, 1}, {InputState::MISSING, 2}};
    for (const InputField& input : INPUT_FIELDS) {
        for (const auto& [state, expected] : states) {
            StatusHealth health = HealthyHealth();
            health.*input.health = state;
            const StatusSnapshot snapshot = EncodeV120Status(health);
            EXPECT_EQ(snapshot.*input.wire, expected) << input.name << " " << static_cast<int>(state);
            EXPECT_EQ(FieldsExcept(snapshot, input.wire), FieldsExcept(healthy, input.wire)) << input.name;
        }
    }
}

TEST(StatusContractTest, EncodeStatusFollowsTheContract) {
    const StatusHealth healthy = HealthyHealth();
    EXPECT_EQ(EncodeStatus(StatusContract::LEGACY, healthy).total_status, 1);
    EXPECT_EQ(EncodeStatus(StatusContract::V1_2_0, healthy).total_status, 0);
    StatusHealth starting;
    EXPECT_EQ(InputsOf(EncodeStatus(StatusContract::LEGACY, starting)), Inputs(0, 0, 0, 0, 0, 0));
    EXPECT_EQ(InputsOf(EncodeStatus(StatusContract::V1_2_0, starting)), Inputs(2, 2, 2, 2, 0, 0));
}

TEST(StatusContractTest, V120ReportsWhatTheMonitorSees) {
    // From a cold start: initializing, no data from any monitored input, the odometry a hard loss
    const StatusMonitor cold{StatusTimeouts(), false};
    const StatusSnapshot start = V120(cold, 0.0);
    EXPECT_EQ(TotalAndExecOf(start), TotalAndExec(7, 0, 0, 0));
    EXPECT_EQ(LossesOf(start), Losses(2, 1, 0, 0));
    EXPECT_EQ(InputsOf(start), Inputs(2, 2, 2, 2, 0, 0));

    // Everything alive: normal
    StatusMonitor monitor{StatusTimeouts(), false};
    MakeHealthy(monitor);
    EXPECT_EQ(TotalAndExecOf(V120(monitor, 1.05)), TotalAndExec(0, 0, 0, 0));
    EXPECT_EQ(LossesOf(V120(monitor, 1.05)), Losses(0, 0, 0, 0));
    EXPECT_EQ(InputsOf(V120(monitor, 1.05)), Inputs(1, 1, 1, 1, 0, 0));

    // The lidar silent past its timeout: a minor fault, still usable, the lidar without data
    EXPECT_EQ(V120(monitor, 1.6).total_status, 2);
    EXPECT_EQ(V120(monitor, 1.6).lidar, 2);

    // A held numeric error: lost
    monitor.MarkError(StatusError::NUMERIC, 1.0);
    EXPECT_EQ(TotalAndExecOf(V120(monitor, 1.05)), TotalAndExec(3, 0, 1, 0));

    // The odometry stalled: lost, a hard loss
    StatusMonitor stalled{StatusTimeouts(), false};
    MakeHealthy(stalled);
    EXPECT_EQ(V120(stalled, 3.0).total_status, 3);
    EXPECT_EQ(V120(stalled, 3.0).odom_error, 2);

    // Monitored scan matching gone quiet: a minor loss and a minor fault
    StatusMonitor matching{StatusTimeouts(), true};
    MakeHealthy(matching, true);
    MarkAllAlive(matching, 2.0);
    EXPECT_EQ(V120(matching, 2.05).matching_error, 1);
    EXPECT_EQ(V120(matching, 2.05).total_status, 2);
}

TEST(StatusContractTest, InitializingIsSevenInV120Only) {
    const std::vector<StatusVerdict> verdicts = {StatusVerdict::UNINITIALIZED, StatusVerdict::LOST,
                                                 StatusVerdict::LOW_QUALITY, StatusVerdict::NORMAL};
    for (const StatusVerdict verdict : verdicts) {
        StatusHealth health = HealthyHealth();
        health.verdict = verdict;
        health.lidar = InputState::MISSING;
        health.pub_error = true;
        StatusHealth initializing = health;
        initializing.initializing = true;

        // v1.2.0: 7 whatever the verdict, every other field as it was
        const StatusSnapshot v120 = EncodeV120Status(initializing);
        EXPECT_EQ(v120.total_status, 7) << static_cast<int>(verdict);
        EXPECT_EQ(FieldsExcept(v120, &StatusSnapshot::total_status),
                  FieldsExcept(EncodeV120Status(health), &StatusSnapshot::total_status))
            << static_cast<int>(verdict);

        // Legacy has no such state: not a single field changes
        EXPECT_EQ(FieldsExcept(EncodeLegacyStatus(initializing), nullptr),
                  FieldsExcept(EncodeLegacyStatus(health), nullptr))
            << static_cast<int>(verdict);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

namespace {

//! Time between samples [s]. A power of two, so every time below is exact and the settle and timeout
//! boundaries can be hit on the dot.
constexpr double STEP = 0.125;

//! The v1.2.0 total_status as (time, value): at the first time looked at, and wherever it changed
using Changes = std::vector<std::pair<double, int>>;

/**
 * Every STEP over [from, to] the lidar, the IMU, the leg odometry and the RTK input arrive; so does the
 * odometry if @p odometry, accepted the way the module accepts it (alive, and initialized from then
 * on). Returns what the v1.2.0 contract reports after each step.
 */
Changes Drive(StatusMonitor& monitor, const double from, const double to, const bool odometry) {
    Changes changes;
    for (double now = from; now <= to; now += STEP) {
        monitor.MarkRx(StatusInput::LIDAR, now);
        monitor.MarkRx(StatusInput::IMU, now);
        monitor.MarkRx(StatusInput::LEG_ODOM, now);
        monitor.MarkRx(StatusInput::RTK, now);
        if (odometry) {
            monitor.MarkOdomAlive(now);
            monitor.SetInitialized();
        }
        const int total = V120(monitor, now).total_status;
        if (changes.empty() || (changes.back().second != total)) {
            changes.emplace_back(now, total);
        }
    }
    return changes;
}

}  // namespace

TEST(InitialPoseHoldTest, InitializingUntilTheOdometryHasFlowedForTheSettleTime) {
    StatusMonitor monitor{StatusTimeouts(), false};
    EXPECT_EQ(Drive(monitor, 1.0, 10.0, true), (Changes{{1.0, 0}}));

    // From the moment the pose arrives ...
    monitor.MarkInitialPose(10.0625);
    EXPECT_EQ(V120(monitor, 10.0625).total_status, 7);
    // ... until 2 s of odometry, counted from the first sample after it
    EXPECT_EQ(Drive(monitor, 10.125, 15.0, true), (Changes{{10.125, 7}, {12.125, 0}}));
}

TEST(InitialPoseHoldTest, OnlyTotalStatusIsHeld) {
    StatusMonitor monitor{StatusTimeouts(), false};
    Drive(monitor, 1.0, 10.0, true);
    monitor.MarkInitialPose(10.0625);
    Drive(monitor, 10.125, 11.0, true);

    // Every input is arriving and nothing has failed: the fields say so, only the verdict waits
    const StatusSnapshot held = V120(monitor, 11.0);
    EXPECT_EQ(TotalAndExecOf(held), TotalAndExec(7, 0, 0, 0));
    EXPECT_EQ(LossesOf(held), Losses(0, 0, 0, 0));
    EXPECT_EQ(InputsOf(held), Inputs(1, 1, 1, 1, 0, 0));

    // The legacy contract has no initializing state
    EXPECT_EQ(Legacy(monitor, 11.0).total_status, TOTAL_NORMAL);
}

TEST(InitialPoseHoldTest, CountStartsWithTheFirstSampleAfterThePose) {
    StatusMonitor monitor{StatusTimeouts(), false};
    Drive(monitor, 1.0, 5.0, true);
    monitor.MarkInitialPose(5.0625);

    // No odometry for 3 s: initializing throughout, also once the odometry counts as stopped
    EXPECT_EQ(Drive(monitor, 5.125, 8.0, false), (Changes{{5.125, 7}}));
    // It comes back: 2 s from its first sample
    EXPECT_EQ(Drive(monitor, 8.125, 12.0, true), (Changes{{8.125, 7}, {10.125, 0}}));
}

TEST(InitialPoseHoldTest, PoseBeforeTheFirstOdometry) {
    StatusMonitor monitor{StatusTimeouts(), false};
    monitor.MarkInitialPose(0.5);
    EXPECT_EQ(Drive(monitor, 0.5, 1.0, false), (Changes{{0.5, 7}}));
    // Without the pose the first sample would have made it normal at once
    EXPECT_EQ(Drive(monitor, 1.125, 5.0, true), (Changes{{1.125, 7}, {3.125, 0}}));
}

TEST(InitialPoseHoldTest, BreakInTheOdometryRestartsTheCount) {
    StatusMonitor monitor{StatusTimeouts(), false};
    Drive(monitor, 1.0, 20.0, true);
    monitor.MarkInitialPose(20.0625);
    EXPECT_EQ(Drive(monitor, 20.125, 20.5, true), (Changes{{20.125, 7}}));
    // Last sample at 20.5: the odometry counts as flowing up to 21.5 (rtk timeout 1 s), then it has stopped
    EXPECT_EQ(Drive(monitor, 20.625, 22.0, false), (Changes{{20.625, 7}}));
    // Had the count gone on, it would have been 0 from 22.125
    EXPECT_EQ(Drive(monitor, 22.125, 26.0, true), (Changes{{22.125, 7}, {24.125, 0}}));
}

TEST(InitialPoseHoldTest, ShortStopWithinTheOdometryTimeoutIsNoBreak) {
    StatusMonitor monitor{StatusTimeouts(), false};
    Drive(monitor, 1.0, 30.0, true);
    monitor.MarkInitialPose(30.0625);
    EXPECT_EQ(Drive(monitor, 30.125, 30.5, true), (Changes{{30.125, 7}}));
    // Silent from 30.625 to 31.375; the next sample at 31.5 is exactly the 1 s timeout after the last
    EXPECT_EQ(Drive(monitor, 30.625, 31.375, false), (Changes{{30.625, 7}}));
    EXPECT_EQ(Drive(monitor, 31.5, 34.0, true), (Changes{{31.5, 7}, {32.125, 0}}));
}

TEST(InitialPoseHoldTest, NewPoseRestartsTheCount) {
    StatusMonitor monitor{StatusTimeouts(), false};
    Drive(monitor, 1.0, 40.0, true);
    monitor.MarkInitialPose(40.0625);
    EXPECT_EQ(Drive(monitor, 40.125, 41.5, true), (Changes{{40.125, 7}}));
    // Had the first pose's count gone on, it would have been 0 from 42.125
    monitor.MarkInitialPose(41.5625);
    EXPECT_EQ(Drive(monitor, 41.625, 45.0, true), (Changes{{41.625, 7}, {43.625, 0}}));
}

TEST(InitialPoseHoldTest, AfterSettlingAStopIsAnOrdinaryLoss) {
    StatusMonitor monitor{StatusTimeouts(), false};
    Drive(monitor, 1.0, 50.0, true);
    monitor.MarkInitialPose(50.0625);
    EXPECT_EQ(Drive(monitor, 50.125, 53.0, true), (Changes{{50.125, 7}, {52.125, 0}}));
    // The odometry stops: lost (3) once the timeout has passed, not initializing again
    EXPECT_EQ(Drive(monitor, 53.125, 56.0, false), (Changes{{53.125, 0}, {54.125, 3}}));
    // And it is normal as soon as it flows again
    EXPECT_EQ(Drive(monitor, 56.125, 57.0, true), (Changes{{56.125, 0}}));
}

TEST(InitialPoseHoldTest, HonoursTheSettleTime) {
    StatusTimeouts timeouts;
    timeouts.initialpose_settle = 0.5;
    StatusMonitor monitor{timeouts, false};
    Drive(monitor, 1.0, 5.0, true);
    monitor.MarkInitialPose(5.0625);
    EXPECT_EQ(Drive(monitor, 5.125, 7.0, true), (Changes{{5.125, 7}, {5.625, 0}}));
}
