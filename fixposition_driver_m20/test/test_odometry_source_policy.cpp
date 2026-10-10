/**
 * @file
 * @brief Tests for the odometry source policy and the status each mode ends up reporting
 *
 * Built from the pure sources alone (the policy header and status_monitor.cpp): no node, no
 * publishers. The Follower below repeats what the module does with each action, so these are tests of
 * the decision and of the status arithmetic it drives — not of the ROS wiring, the published messages
 * or the stamp retiming.
 */

/* LIBC/STL */
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include "fixposition_driver_m20/odometry_source_policy.hpp"
#include "fixposition_driver_m20/status_monitor.hpp"

namespace {

using fixposition::m20::DecideOdomInput;
using fixposition::m20::ERR_ERROR;
using fixposition::m20::ERR_OK;
using fixposition::m20::INPUT_MISSING;
using fixposition::m20::INPUT_OK;
using fixposition::m20::OdomFeed;
using fixposition::m20::OdomInputAction;
using fixposition::m20::OdomSource;
using fixposition::m20::StatusError;
using fixposition::m20::StatusInput;
using fixposition::m20::StatusMonitor;
using fixposition::m20::StatusTimeouts;
using fixposition::m20::TOTAL_LOST;
using fixposition::m20::TOTAL_LOW_QUALITY;
using fixposition::m20::TOTAL_NORMAL;
using fixposition::m20::TOTAL_UNINITIALIZED;

//! Time between samples [s]. A power of two, so every time below is exact and the timeout boundaries
//! can be hit on the dot.
constexpr double STEP = 0.125;

//! The default odometry timeout (StatusTimeouts::rtk) and error hold, spelled out for the boundaries
constexpr double ODOM_TIMEOUT = 1.0;
constexpr double ERROR_HOLD = 5.0;

//! (total_status, loss_val.odom_error, input_val.rtk), as ints so that a failure prints numbers
using Triple = std::tuple<int, int, int>;

const Triple NORMAL{TOTAL_NORMAL, ERR_OK, INPUT_OK};                                         // (1, 0, 1)
const Triple LOW_QUALITY{TOTAL_LOW_QUALITY, ERR_OK, INPUT_OK};                               // (2, 0, 1)
const Triple LOST_SENSOR_ALIVE{TOTAL_LOST, ERR_ERROR, INPUT_OK};                             // (3, 1, 1)
const Triple LOST_NOTHING_ARRIVING{TOTAL_LOST, ERR_ERROR, INPUT_MISSING};                    // (3, 1, 0)
const Triple UNINITIALIZED_SENSOR_ALIVE{TOTAL_UNINITIALIZED, ERR_ERROR, INPUT_OK};           // (0, 1, 1)
const Triple UNINITIALIZED_NOTHING_ARRIVING{TOTAL_UNINITIALIZED, ERR_ERROR, INPUT_MISSING};  // (0, 1, 0)

//! The parts of an odometry sample that tell the two feeds apart
struct Sample {
    bool valid = true;
    double stamp = 0.0;     //!< the sample's own stamp [s]
    double position = 0.0;  //!< one axis is enough
    double velocity = 0.0;

    bool Finite() const { return std::isfinite(position) && std::isfinite(velocity); }
};

//! What the SLAM pipeline publishes on /ODOM at steady time @p now
Sample PipelineSample(const double now) {
    Sample sample;
    sample.stamp = 1000.0 + now;
    sample.position = now;
    sample.velocity = 0.5;
    return sample;
}

//! What the sensor reports at steady time @p now: another clock, another pose, another velocity
Sample SensorSample(const double now) {
    Sample sample;
    sample.stamp = 500.0 + now;
    sample.position = 100.0 - now;
    sample.velocity = -3.0;
    return sample;
}

/**
 * @brief Stand-in for the module's odometry entry
 *
 * The real policy and the real status monitor, with the publishers, the stamp reference and the
 * counter reduced to plain records.
 */
class Follower {
   public:
    explicit Follower(const OdomSource mode, const bool alias_enabled = true)
        : monitor{StatusTimeouts(), false}, mode_{mode}, alias_enabled_{alias_enabled} {}

    OdomInputAction Feed(const OdomFeed feed, const Sample& sample, const double now) {
        const OdomInputAction action = DecideOdomInput(mode_, feed, sample.valid, sample.Finite());
        switch (action) {
            case OdomInputAction::IGNORE:
                break;
            case OdomInputAction::RTK_HEALTH_ONLY:
                monitor.MarkRx(StatusInput::RTK, now);
                break;
            case OdomInputAction::SELECTED_NUMERIC_ERROR:
                monitor.MarkError(StatusError::NUMERIC, now);
                break;
            case OdomInputAction::SELECTED:
                if (alias_enabled_) {
                    alias_stamps.push_back(sample.stamp);
                }
                reference_stamp = sample.stamp;
                reference_velocity = sample.velocity;
                have_reference = true;
                monitor.MarkRx(StatusInput::RTK, now);
                monitor.MarkOdomAlive(now);
                monitor.SetInitialized();
                count++;
                break;
        }
        return action;
    }

    //! The status triple at @p now, with the lidar, the IMU and the leg odometry kept alive so that
    //! only the odometry feeds decide it
    Triple StatusAt(const double now) {
        monitor.MarkRx(StatusInput::LIDAR, now);
        monitor.MarkRx(StatusInput::IMU, now);
        monitor.MarkRx(StatusInput::LEG_ODOM, now);
        const auto snapshot = monitor.Compute(now);
        return Triple{snapshot.total_status, snapshot.odom_error, snapshot.rtk};
    }

    //! True while nothing has taken the selected path
    bool Untouched() const { return alias_stamps.empty() && !have_reference && (count == 0) && !monitor.Initialized(); }

    StatusMonitor monitor;
    std::vector<double> alias_stamps;  //!< stamps that went out on the handler alias
    double reference_stamp = 0.0;      //!< the stamp the cloud and the status would take
    double reference_velocity = 0.0;   //!< the velocity the deskew would use
    bool have_reference = false;
    uint64_t count = 0;

   private:
    OdomSource mode_;
    bool alias_enabled_;
};

}  // namespace

TEST(OdometrySourcePolicyTest, ModeFeedMatrix) {
    EXPECT_EQ(DecideOdomInput(OdomSource::FPA_ODOMENU, OdomFeed::SENSOR_ENU, true, true), OdomInputAction::SELECTED);
    EXPECT_EQ(DecideOdomInput(OdomSource::FPA_ODOMENU, OdomFeed::SUBSCRIBED_ODOM, true, true), OdomInputAction::IGNORE);
    EXPECT_EQ(DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SENSOR_ENU, true, true),
              OdomInputAction::RTK_HEALTH_ONLY);
    EXPECT_EQ(DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SUBSCRIBED_ODOM, true, true), OdomInputAction::SELECTED);

    // No fallback: whatever the sample looks like, the feed a mode did not select never takes the
    // selected path, not even its error branch
    for (const bool valid : {false, true}) {
        for (const bool finite : {false, true}) {
            EXPECT_EQ(DecideOdomInput(OdomSource::FPA_ODOMENU, OdomFeed::SUBSCRIBED_ODOM, valid, finite),
                      OdomInputAction::IGNORE);
            const OdomInputAction sensor = DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SENSOR_ENU, valid, finite);
            EXPECT_NE(sensor, OdomInputAction::SELECTED);
            EXPECT_NE(sensor, OdomInputAction::SELECTED_NUMERIC_ERROR);
        }
    }
}

TEST(OdometrySourcePolicyTest, HealthOnlyRequiresValidFinite) {
    // The sensor's sample counts as a heartbeat only when it is both valid and finite
    EXPECT_EQ(DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SENSOR_ENU, true, true),
              OdomInputAction::RTK_HEALTH_ONLY);
    EXPECT_EQ(DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SENSOR_ENU, false, true), OdomInputAction::IGNORE);
    EXPECT_EQ(DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SENSOR_ENU, true, false), OdomInputAction::IGNORE);
    EXPECT_EQ(DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SENSOR_ENU, false, false), OdomInputAction::IGNORE);

    // A non-finite value from the selected source is still that source's error, in either mode; an
    // invalid sample from it is dropped without one
    EXPECT_EQ(DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SUBSCRIBED_ODOM, true, false),
              OdomInputAction::SELECTED_NUMERIC_ERROR);
    EXPECT_EQ(DecideOdomInput(OdomSource::FPA_ODOMENU, OdomFeed::SENSOR_ENU, true, false),
              OdomInputAction::SELECTED_NUMERIC_ERROR);
    EXPECT_EQ(DecideOdomInput(OdomSource::SUBSCRIBE, OdomFeed::SUBSCRIBED_ODOM, false, false), OdomInputAction::IGNORE);
    EXPECT_EQ(DecideOdomInput(OdomSource::FPA_ODOMENU, OdomFeed::SENSOR_ENU, false, false), OdomInputAction::IGNORE);

    // Unusable sensor samples leave no trace at all: nothing published, no reference, no
    // initialization, no count, no RTK mark and no numeric error
    Follower follower{OdomSource::SUBSCRIBE};
    Sample invalid = SensorSample(1.0);
    invalid.valid = false;
    Sample not_finite = SensorSample(1.0);
    not_finite.position = std::numeric_limits<double>::quiet_NaN();
    Sample invalid_and_not_finite = not_finite;
    invalid_and_not_finite.valid = false;
    EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, invalid, 1.0), OdomInputAction::IGNORE);
    EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, not_finite, 1.0), OdomInputAction::IGNORE);
    EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, invalid_and_not_finite, 1.0), OdomInputAction::IGNORE);
    EXPECT_TRUE(follower.Untouched());
    EXPECT_FALSE(follower.monitor.EverReceived(StatusInput::RTK));
    EXPECT_EQ(follower.monitor.Compute(1.0).numeric_error, ERR_OK);
    EXPECT_EQ(follower.StatusAt(1.0), UNINITIALIZED_NOTHING_ARRIVING);

    // With the pipeline healthy, the sensor's NaN does not cost the localization its status ...
    EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(1.0), 1.0), OdomInputAction::SELECTED);
    EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, not_finite, 1.0), OdomInputAction::IGNORE);
    EXPECT_EQ(follower.monitor.Compute(1.0).numeric_error, ERR_OK);
    EXPECT_EQ(follower.StatusAt(1.0), NORMAL);

    // ... while the pipeline's own NaN does, and is not published or counted either
    Sample pipeline_not_finite = PipelineSample(1.0 + STEP);
    pipeline_not_finite.velocity = std::numeric_limits<double>::infinity();
    EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, pipeline_not_finite, 1.0 + STEP),
              OdomInputAction::SELECTED_NUMERIC_ERROR);
    EXPECT_EQ(follower.monitor.Compute(1.0 + STEP).numeric_error, ERR_ERROR);
    EXPECT_EQ(follower.monitor.Compute(1.0 + STEP).total_status, TOTAL_LOST);
    EXPECT_EQ(follower.count, 1u);
    EXPECT_EQ(follower.alias_stamps.size(), 1u);
    EXPECT_EQ(follower.reference_stamp, PipelineSample(1.0).stamp);
}

TEST(OdometrySourcePolicyTest, InterleavingSelectsOnlyPipeline) {
    // Which feed arrives when; the sensor's stamps also run backwards against the pipeline's
    const std::vector<OdomFeed> order = {
        OdomFeed::SENSOR_ENU,      OdomFeed::SUBSCRIBED_ODOM, OdomFeed::SENSOR_ENU,      OdomFeed::SENSOR_ENU,
        OdomFeed::SUBSCRIBED_ODOM, OdomFeed::SUBSCRIBED_ODOM, OdomFeed::SENSOR_ENU,      OdomFeed::SUBSCRIBED_ODOM,
        OdomFeed::SENSOR_ENU,      OdomFeed::SENSOR_ENU,      OdomFeed::SUBSCRIBED_ODOM, OdomFeed::SENSOR_ENU,
    };

    // The handler alias being disabled changes what is published, not which feed is followed
    for (const bool alias_enabled : {true, false}) {
        Follower follower{OdomSource::SUBSCRIBE, alias_enabled};
        std::vector<double> pipeline_stamps;
        double last_pipeline_velocity = 0.0;

        for (std::size_t ix = 0; ix < order.size(); ix++) {
            const double now = 1.0 + (static_cast<double>(ix) * STEP);
            if (order[ix] == OdomFeed::SUBSCRIBED_ODOM) {
                Sample sample = PipelineSample(now);
                sample.velocity = 0.5 + now;
                EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, sample, now), OdomInputAction::SELECTED);
                pipeline_stamps.push_back(sample.stamp);
                last_pipeline_velocity = sample.velocity;
            } else {
                Sample sample = SensorSample(now);
                sample.stamp = 500.0 - now;
                EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, sample, now), OdomInputAction::RTK_HEALTH_ONLY);
            }

            // After every sample, of either feed, the reference is the newest pipeline sample (or
            // nothing, before the first one) and only pipeline samples have been counted
            EXPECT_EQ(follower.have_reference, !pipeline_stamps.empty());
            EXPECT_EQ(follower.count, pipeline_stamps.size());
            if (!pipeline_stamps.empty()) {
                EXPECT_EQ(follower.reference_stamp, pipeline_stamps.back());
                EXPECT_EQ(follower.reference_velocity, last_pipeline_velocity);
            }
        }

        if (alias_enabled) {
            EXPECT_EQ(follower.alias_stamps, pipeline_stamps);
        } else {
            EXPECT_TRUE(follower.alias_stamps.empty());
        }
        EXPECT_EQ(pipeline_stamps.size(), 5u);
    }
}

TEST(OdometrySourcePolicyTest, SubscribeTimeoutWithSensorAlive) {
    Follower follower{OdomSource::SUBSCRIBE};

    // Both feeds flow ...
    const double stopped = 2.0;
    uint64_t pipeline_count = 0;
    for (double now = 1.0; now <= stopped; now += STEP) {
        follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now);
        follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now);
        pipeline_count++;
        EXPECT_EQ(follower.StatusAt(now), NORMAL);
    }

    // ... then the pipeline stops and the sensor carries on. The odometry stays fresh up to and
    // including the timeout (the last pass of this loop is exactly on it) ...
    for (double now = stopped + STEP; now <= stopped + ODOM_TIMEOUT; now += STEP) {
        EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now), OdomInputAction::RTK_HEALTH_ONLY);
        EXPECT_EQ(follower.StatusAt(now), NORMAL) << now;
    }
    // ... and is lost from the next sample on, with the RTK input still arriving
    for (double now = stopped + ODOM_TIMEOUT + STEP; now <= stopped + (3.0 * ODOM_TIMEOUT); now += STEP) {
        EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now), OdomInputAction::RTK_HEALTH_ONLY);
        EXPECT_EQ(follower.StatusAt(now), LOST_SENSOR_ALIVE) << now;
    }

    // The sensor did not take over anything on the way
    EXPECT_EQ(follower.count, pipeline_count);
    EXPECT_EQ(follower.alias_stamps.size(), pipeline_count);
    EXPECT_EQ(follower.reference_stamp, PipelineSample(stopped).stamp);
    EXPECT_EQ(follower.reference_velocity, PipelineSample(stopped).velocity);
}

TEST(OdometrySourcePolicyTest, SubscribeColdStartSensorOnly) {
    Follower follower{OdomSource::SUBSCRIBE};
    EXPECT_EQ(follower.StatusAt(1.0), UNINITIALIZED_NOTHING_ARRIVING);

    // The sensor has a valid ENU pose long before the pipeline publishes anything: that is an
    // input arriving, not a localization
    for (double now = 1.0; now <= 1.0 + (3.0 * ODOM_TIMEOUT); now += STEP) {
        EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now), OdomInputAction::RTK_HEALTH_ONLY);
        EXPECT_EQ(follower.StatusAt(now), UNINITIALIZED_SENSOR_ALIVE) << now;
    }
    EXPECT_TRUE(follower.Untouched());

    // The first /ODOM from the pipeline is what initializes it
    EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(5.0), 5.0), OdomInputAction::SELECTED);
    EXPECT_EQ(follower.StatusAt(5.0), NORMAL);
    EXPECT_EQ(follower.count, 1u);
}

TEST(OdometrySourcePolicyTest, SubscribeNoValidEnu) {
    // Indoors from a cold start: the sensor reports all along, but never a valid ENU pose. The
    // pipeline localizes on its own and the status is normal, RTK input included.
    Follower follower{OdomSource::SUBSCRIBE};
    for (double now = 1.0; now <= 1.0 + (3.0 * ODOM_TIMEOUT); now += STEP) {
        Sample no_fix = SensorSample(now);
        no_fix.valid = false;
        EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, no_fix, now), OdomInputAction::IGNORE);
        EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now), OdomInputAction::SELECTED);
        EXPECT_EQ(follower.StatusAt(now), NORMAL) << now;
    }
}

TEST(OdometrySourcePolicyTest, SubscribeTcpDown) {
    Follower follower{OdomSource::SUBSCRIBE};
    for (double now = 1.0; now <= 2.0; now += STEP) {
        follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now);
        follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now);
    }
    EXPECT_EQ(follower.StatusAt(2.0), NORMAL);

    // The sensor's connection drops: nothing at all from it any more, well past the RTK timeout,
    // while the pipeline keeps publishing
    for (double now = 2.0 + STEP; now <= 2.0 + (3.0 * ODOM_TIMEOUT); now += STEP) {
        EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now), OdomInputAction::SELECTED);
        EXPECT_EQ(follower.StatusAt(now), NORMAL) << now;
    }
}

TEST(OdometrySourcePolicyTest, SubscribeBothDownAndRecover) {
    Follower follower{OdomSource::SUBSCRIBE};
    const double stopped = 2.0;
    for (double now = 1.0; now <= stopped; now += STEP) {
        follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now);
        follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now);
    }
    EXPECT_EQ(follower.StatusAt(stopped), NORMAL);

    // Both feeds stop: fresh up to and including the timeout, then lost with no RTK input
    EXPECT_EQ(follower.StatusAt(stopped + ODOM_TIMEOUT), NORMAL);
    EXPECT_EQ(follower.StatusAt(stopped + ODOM_TIMEOUT + STEP), LOST_NOTHING_ARRIVING);

    // While the odometry is away the cloud path flags its stale stamp reference; mark that the way
    // the module does, to see the hold outlive the outage
    const double flagged = 4.0;
    follower.monitor.MarkError(StatusError::TIMESTAMP, flagged);
    EXPECT_EQ(follower.StatusAt(flagged), LOST_NOTHING_ARRIVING);

    // The sensor alone coming back restores the RTK input and nothing else
    EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(4.5), 4.5), OdomInputAction::RTK_HEALTH_ONLY);
    EXPECT_EQ(follower.StatusAt(4.5), LOST_SENSOR_ALIVE);

    // The pipeline coming back restores the odometry at once. The flagged error keeps the verdict at
    // low quality for its hold (the last pass of this loop is exactly on its end) ...
    for (double now = 5.0; now <= flagged + ERROR_HOLD; now += STEP) {
        EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now), OdomInputAction::SELECTED);
        follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now);
        EXPECT_EQ(follower.StatusAt(now), LOW_QUALITY) << now;
        EXPECT_EQ(follower.monitor.Compute(now).timestamp_error, ERR_ERROR) << now;
    }
    // ... and then it is normal again
    for (double now = flagged + ERROR_HOLD + STEP; now <= flagged + ERROR_HOLD + 1.0; now += STEP) {
        EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now), OdomInputAction::SELECTED);
        follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now);
        EXPECT_EQ(follower.StatusAt(now), NORMAL) << now;
        EXPECT_EQ(follower.monitor.Compute(now).timestamp_error, ERR_OK) << now;
    }
}

TEST(OdometrySourcePolicyTest, FpaModeKeepsSelectedPath) {
    // Positioning-only mode: the sensor's ENU output is the odometry, with everything that entails
    Follower follower{OdomSource::FPA_ODOMENU};
    std::vector<double> sensor_stamps;
    for (double now = 1.0; now <= 2.0; now += STEP) {
        EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, SensorSample(now), now), OdomInputAction::SELECTED);
        sensor_stamps.push_back(SensorSample(now).stamp);
        // An /ODOM from elsewhere has no business here, and is ignored outright
        EXPECT_EQ(follower.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now), OdomInputAction::IGNORE);
        EXPECT_EQ(follower.StatusAt(now), NORMAL);
    }
    EXPECT_EQ(follower.alias_stamps, sensor_stamps);
    EXPECT_EQ(follower.count, sensor_stamps.size());
    EXPECT_EQ(follower.reference_stamp, SensorSample(2.0).stamp);
    EXPECT_EQ(follower.reference_velocity, SensorSample(2.0).velocity);

    // An invalid sample is dropped silently; a non-finite one is a numeric error, which is lost
    Sample invalid = SensorSample(2.0 + STEP);
    invalid.valid = false;
    EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, invalid, 2.0 + STEP), OdomInputAction::IGNORE);
    EXPECT_EQ(follower.StatusAt(2.0 + STEP), NORMAL);
    Sample not_finite = SensorSample(2.0 + STEP);
    not_finite.position = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(follower.Feed(OdomFeed::SENSOR_ENU, not_finite, 2.0 + STEP), OdomInputAction::SELECTED_NUMERIC_ERROR);
    EXPECT_EQ(follower.monitor.Compute(2.0 + STEP).numeric_error, ERR_ERROR);
    EXPECT_EQ(follower.monitor.Compute(2.0 + STEP).total_status, TOTAL_LOST);
    EXPECT_EQ(follower.count, sensor_stamps.size());

    // The sensor going quiet is the odometry going quiet: an /ODOM from elsewhere cannot stand in for
    // it, neither as odometry nor as the RTK input
    Follower quiet{OdomSource::FPA_ODOMENU};
    quiet.Feed(OdomFeed::SENSOR_ENU, SensorSample(1.0), 1.0);
    for (double now = 1.0 + STEP; now <= 1.0 + ODOM_TIMEOUT; now += STEP) {
        quiet.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now);
        EXPECT_EQ(quiet.StatusAt(now), NORMAL) << now;
    }
    for (double now = 1.0 + ODOM_TIMEOUT + STEP; now <= 1.0 + (2.0 * ODOM_TIMEOUT); now += STEP) {
        quiet.Feed(OdomFeed::SUBSCRIBED_ODOM, PipelineSample(now), now);
        EXPECT_EQ(quiet.StatusAt(now), LOST_NOTHING_ARRIVING) << now;
    }
    EXPECT_EQ(quiet.count, 1u);
}
