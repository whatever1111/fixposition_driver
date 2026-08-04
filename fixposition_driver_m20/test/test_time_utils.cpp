/**
 * @file
 * @brief Tests for the stamp helpers and the sensor-to-host clock retiming
 */

/* LIBC/STL */
#include <cmath>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include "fixposition_driver_m20/time_utils.hpp"

namespace {
using fixposition::m20::HostClockRetimer;
using fixposition::m20::Stamp;
}  // namespace

TEST(StampTest, RoundTrips) {
    const Stamp stamp = Stamp::FromSec(1785000000.25);
    EXPECT_EQ(stamp.sec, 1785000000);
    EXPECT_EQ(stamp.nsec, 250000000u);
    EXPECT_NEAR(stamp.ToSec(), 1785000000.25, 1e-6);
}

TEST(StampTest, CarriesOverAtTheNanosecondBoundary) {
    // A value that rounds up to a full second must not leave nsec at 1e9
    const Stamp stamp = Stamp::FromSec(std::nextafter(100.0, 101.0));
    EXPECT_EQ(stamp.sec, 100);
    EXPECT_LT(stamp.nsec, 1000000000u);
}

TEST(StampTest, ClampsNonsense) {
    EXPECT_EQ(Stamp::FromSec(-1.0).sec, 0);
    EXPECT_EQ(Stamp::FromSec(std::nan("")).sec, 0);
    EXPECT_EQ(Stamp::FromSec(std::nan("")).nsec, 0u);
}

TEST(StampTest, ComparesForTheSharedStampContract) {
    EXPECT_TRUE(Stamp::FromSec(1785000000.5) == Stamp::FromSec(1785000000.5));
    EXPECT_FALSE(Stamp::FromSec(1785000000.5) == Stamp::FromSec(1785000000.6));
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(HostClockRetimerTest, MovesStampsOntoTheHostAxis) {
    HostClockRetimer retimer{0.5};
    // The sensor's clock runs 80 s behind the host's, plus a little transport delay
    const double offset = 80.0;
    double corrected = 0.0;
    for (int i = 0; i < 20; i++) {
        const double host_now = 1000.0 + (0.1 * i);
        corrected = retimer.Apply(host_now - offset, host_now, nullptr);
    }
    EXPECT_NEAR(corrected, 1000.0 + (0.1 * 19), 1e-6);
    EXPECT_NEAR(retimer.Offset(), offset, 1e-6);
}

TEST(HostClockRetimerTest, IsANoOpWhenTheClocksAgree) {
    HostClockRetimer retimer{0.5};
    double corrected = 0.0;
    for (int i = 0; i < 20; i++) {
        const double host_now = 1000.0 + (0.1 * i);
        corrected = retimer.Apply(host_now, host_now, nullptr);
    }
    EXPECT_NEAR(corrected, 1000.0 + (0.1 * 19), 1e-9);
    EXPECT_NEAR(retimer.Offset(), 0.0, 1e-9);
}

TEST(HostClockRetimerTest, RidesOutASingleOutlier) {
    HostClockRetimer retimer{10.0};  // wide threshold: the outlier is not treated as a jump
    for (int i = 0; i < 20; i++) {
        const double host_now = 1000.0 + (0.1 * i);
        retimer.Apply(host_now, host_now, nullptr);
    }
    // One late message must not drag the estimate
    retimer.Apply(1002.0 - 1.0, 1002.0, nullptr);
    EXPECT_NEAR(retimer.Offset(), 0.0, 1e-6);
}

TEST(HostClockRetimerTest, ResetsWhenTheSensorClockSteps) {
    HostClockRetimer retimer{0.5};
    for (int i = 0; i < 20; i++) {
        const double host_now = 1000.0 + (0.1 * i);
        retimer.Apply(host_now, host_now, nullptr);
    }
    ASSERT_NEAR(retimer.Offset(), 0.0, 1e-9);

    // The sensor's clock jumps 60 s back; the old offset describes a clock that no longer exists
    bool jumped = false;
    const double corrected = retimer.Apply(1002.0 - 60.0, 1002.0, &jumped);
    EXPECT_TRUE(jumped);
    EXPECT_EQ(retimer.Size(), 1u);
    EXPECT_NEAR(corrected, 1002.0, 1e-9);
}

TEST(HostClockRetimerTest, KeepsTheWindowBounded) {
    HostClockRetimer retimer{0.5, 5};
    for (int i = 0; i < 50; i++) {
        retimer.Apply(1000.0, 1000.0, nullptr);
    }
    EXPECT_EQ(retimer.Size(), 5u);
}

TEST(HostClockRetimerTest, PassesNonFiniteStampsThrough) {
    HostClockRetimer retimer{0.5};
    EXPECT_TRUE(std::isnan(retimer.Apply(std::nan(""), 1000.0, nullptr)));
    EXPECT_EQ(retimer.Size(), 0u);
}
