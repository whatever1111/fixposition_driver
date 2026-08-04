/**
 * @file
 * @brief Tests for the deskewing numerics
 */

/* LIBC/STL */
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include "fixposition_driver_m20/deskew.hpp"

namespace {

using fixposition::m20::CloudLayout;
using fixposition::m20::DeskewCloudInPlace;
using fixposition::m20::DeskewOptions;
using fixposition::m20::DetectPointTimeMode;
using fixposition::m20::ImuHistory;
using fixposition::m20::PointFieldDesc;
using fixposition::m20::PointTimeMode;
using fixposition::m20::ReadPointTime;
using fixposition::m20::RotVecToMatrix;

//! The M20 lidar's layout: x y z intensity (f32), ring (u16), timestamp (f64), 26 bytes per point
constexpr uint32_t M20_POINT_STEP = 26;

std::vector<PointFieldDesc> M20Fields() {
    return {
        {"x", 0, fixposition::m20::POINT_FIELD_FLOAT32, 1},
        {"y", 4, fixposition::m20::POINT_FIELD_FLOAT32, 1},
        {"z", 8, fixposition::m20::POINT_FIELD_FLOAT32, 1},
        {"intensity", 12, fixposition::m20::POINT_FIELD_FLOAT32, 1},
        {"ring", 16, fixposition::m20::POINT_FIELD_UINT16, 1},
        {"timestamp", 18, fixposition::m20::POINT_FIELD_FLOAT64, 1},
    };
}

//! Build a packed cloud from (x, y, z, absolute time) tuples
std::vector<uint8_t> MakeCloud(const std::vector<std::array<double, 4>>& points) {
    std::vector<uint8_t> data(points.size() * M20_POINT_STEP, 0);
    for (size_t i = 0; i < points.size(); i++) {
        uint8_t* p = data.data() + (i * M20_POINT_STEP);
        const float x = static_cast<float>(points[i][0]);
        const float y = static_cast<float>(points[i][1]);
        const float z = static_cast<float>(points[i][2]);
        const double t = points[i][3];
        std::memcpy(p + 0, &x, sizeof(x));
        std::memcpy(p + 4, &y, sizeof(y));
        std::memcpy(p + 8, &z, sizeof(z));
        std::memcpy(p + 18, &t, sizeof(t));
    }
    return data;
}

float PointX(const std::vector<uint8_t>& data, const size_t i) {
    float v = 0.0f;
    std::memcpy(&v, data.data() + (i * M20_POINT_STEP), sizeof(v));
    return v;
}

float PointY(const std::vector<uint8_t>& data, const size_t i) {
    float v = 0.0f;
    std::memcpy(&v, data.data() + (i * M20_POINT_STEP) + 4, sizeof(v));
    return v;
}

//! Gyro history with a constant body rate about z, covering [t0, t1]
ImuHistory ConstantYawRate(const double t0, const double t1, const double rate) {
    ImuHistory imu{5.0};
    for (double t = t0; t <= (t1 + 1e-9); t += 0.005) {
        imu.Add(t, Eigen::Vector3d(0.0, 0.0, rate));
    }
    return imu;
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------

TEST(CloudLayoutTest, FindsM20Layout) {
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    EXPECT_TRUE(layout.valid);
    EXPECT_TRUE(layout.has_time);
    EXPECT_EQ(layout.x_offset, 0u);
    EXPECT_EQ(layout.y_offset, 4u);
    EXPECT_EQ(layout.z_offset, 8u);
    EXPECT_EQ(layout.time_offset, 18u);
    EXPECT_EQ(layout.time_datatype, fixposition::m20::POINT_FIELD_FLOAT64);
    EXPECT_EQ(layout.time_field_name, "timestamp");
}

TEST(CloudLayoutTest, RejectsMissingCoordinate) {
    std::vector<PointFieldDesc> fields = M20Fields();
    fields.erase(fields.begin() + 2);  // drop z
    const CloudLayout layout = CloudLayout::FromFields(fields, M20_POINT_STEP);
    EXPECT_FALSE(layout.valid);
    EXPECT_TRUE(layout.has_time);  // the time field is still there
}

TEST(CloudLayoutTest, RejectsNonFloat32Coordinates) {
    std::vector<PointFieldDesc> fields = {
        {"x", 0, fixposition::m20::POINT_FIELD_FLOAT64, 1},
        {"y", 8, fixposition::m20::POINT_FIELD_FLOAT64, 1},
        {"z", 16, fixposition::m20::POINT_FIELD_FLOAT64, 1},
    };
    EXPECT_FALSE(CloudLayout::FromFields(fields, 24).valid);
}

TEST(CloudLayoutTest, RejectsFieldsThatDoNotFitThePoint) {
    std::vector<PointFieldDesc> fields = M20Fields();
    const CloudLayout layout = CloudLayout::FromFields(fields, 20);  // timestamp would run past the point
    EXPECT_TRUE(layout.valid);
    EXPECT_FALSE(layout.has_time);
}

TEST(CloudLayoutTest, NoTimeFieldWithoutACandidateName) {
    std::vector<PointFieldDesc> fields = {
        {"x", 0, fixposition::m20::POINT_FIELD_FLOAT32, 1},
        {"y", 4, fixposition::m20::POINT_FIELD_FLOAT32, 1},
        {"z", 8, fixposition::m20::POINT_FIELD_FLOAT32, 1},
        {"intensity", 12, fixposition::m20::POINT_FIELD_FLOAT32, 1},
    };
    const CloudLayout layout = CloudLayout::FromFields(fields, 16);
    EXPECT_TRUE(layout.valid);
    EXPECT_FALSE(layout.has_time);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(PointTimeTest, ReadsIntegerNanosecondFields) {
    CloudLayout layout;
    layout.has_time = true;
    layout.time_offset = 0;
    layout.time_datatype = fixposition::m20::POINT_FIELD_UINT32;
    const uint32_t nanos = 1500000;  // 1.5 ms
    std::vector<uint8_t> buffer(4, 0);
    std::memcpy(buffer.data(), &nanos, sizeof(nanos));
    EXPECT_NEAR(ReadPointTime(buffer.data(), layout), 0.0015, 1e-12);
}

TEST(PointTimeTest, DetectsAbsoluteTimes) {
    const double t_scan = 1785000000.0;
    const auto data = MakeCloud({{1.0, 0.0, 0.0, t_scan}, {1.0, 0.0, 0.0, t_scan + 0.05}});
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    EXPECT_EQ(DetectPointTimeMode(data.data(), M20_POINT_STEP, 2, layout, t_scan), PointTimeMode::ABSOLUTE);
}

TEST(PointTimeTest, DetectsScanRelativeTimes) {
    const double t_scan = 1785000000.0;
    const auto data = MakeCloud({{1.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.05}});
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    EXPECT_EQ(DetectPointTimeMode(data.data(), M20_POINT_STEP, 2, layout, t_scan), PointTimeMode::RELATIVE);
}

TEST(PointTimeTest, RejectsNonsenseTimes) {
    const double t_scan = 1785000000.0;
    // Times that are neither near the scan stamp nor small: a different epoch, say
    const auto data = MakeCloud({{1.0, 0.0, 0.0, 1000.0}, {1.0, 0.0, 0.0, 1000.1}});
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    EXPECT_EQ(DetectPointTimeMode(data.data(), M20_POINT_STEP, 2, layout, t_scan), PointTimeMode::UNKNOWN);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ImuHistoryTest, IntegratesAConstantRate) {
    const ImuHistory imu = ConstantYawRate(100.0, 101.0, 0.5);
    const Eigen::Vector3d rotvec = imu.RotVec(100.0, 100.2);
    EXPECT_NEAR(rotvec.x(), 0.0, 1e-9);
    EXPECT_NEAR(rotvec.y(), 0.0, 1e-9);
    EXPECT_NEAR(rotvec.z(), 0.1, 1e-6);
}

TEST(ImuHistoryTest, ClampsOutsideTheSampleRange) {
    const ImuHistory imu = ConstantYawRate(100.0, 100.5, 0.5);
    // Beyond the newest sample the integral is held, not extrapolated
    EXPECT_NEAR(imu.RotVec(100.0, 200.0).z(), imu.RotVec(100.0, 100.5).z(), 1e-9);
}

TEST(ImuHistoryTest, EmptyHistoryIsZero) {
    const ImuHistory imu{2.0};
    EXPECT_TRUE(imu.Empty());
    EXPECT_TRUE(imu.RotVec(1.0, 2.0).isZero());
}

TEST(ImuHistoryTest, DropsOutOfOrderSamplesAndAgesOut) {
    ImuHistory imu{0.1};
    imu.Add(10.0, Eigen::Vector3d(0.0, 0.0, 1.0));
    imu.Add(9.0, Eigen::Vector3d(0.0, 0.0, 1.0));  // out of order, ignored
    EXPECT_EQ(imu.Size(), 1u);
    imu.Add(10.5, Eigen::Vector3d(0.0, 0.0, 1.0));  // pushes the first out of the window
    EXPECT_EQ(imu.Size(), 1u);
    EXPECT_NEAR(imu.NewestTime(), 10.5, 1e-12);
}

TEST(RotVecToMatrixTest, ZeroIsIdentity) {
    EXPECT_TRUE(RotVecToMatrix(Eigen::Vector3d::Zero()).isIdentity());
    EXPECT_TRUE(RotVecToMatrix(Eigen::Vector3d(std::nan(""), 0.0, 0.0)).isIdentity());
}

TEST(RotVecToMatrixTest, RotatesAboutZ) {
    const Eigen::Vector3d rotated = RotVecToMatrix(Eigen::Vector3d(0.0, 0.0, M_PI / 2.0)) * Eigen::Vector3d(1, 0, 0);
    EXPECT_NEAR(rotated.x(), 0.0, 1e-12);
    EXPECT_NEAR(rotated.y(), 1.0, 1e-12);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(DeskewTest, RotatesPointsIntoTheReferencePose) {
    const double t_ref = 1785000000.0;
    const double rate = 1.0;  // rad/s about z
    // Two points on the x axis, 0.1 s apart; the second was seen after the body turned by 0.1 rad
    auto data = MakeCloud({{1.0, 0.0, 0.0, t_ref}, {1.0, 0.0, 0.0, t_ref + 0.1}});
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    const ImuHistory imu = ConstantYawRate(t_ref - 0.5, t_ref + 0.5, rate);

    DeskewOptions opts;
    opts.use_linear_velocity = false;
    const auto stats = DeskewCloudInPlace(data.data(), data.size(), M20_POINT_STEP, 2, layout, imu, t_ref, t_ref,
                                          Eigen::Vector3d::Zero(), opts);

    EXPECT_TRUE(stats.deskewed);
    EXPECT_EQ(stats.points, 2u);
    // The point captured at the reference time is untouched
    EXPECT_NEAR(PointX(data, 0), 1.0f, 1e-5);
    EXPECT_NEAR(PointY(data, 0), 0.0f, 1e-5);
    // The later point is rotated back by the 0.1 rad the body turned
    EXPECT_NEAR(PointX(data, 1), static_cast<float>(std::cos(0.1)), 1e-4);
    EXPECT_NEAR(PointY(data, 1), static_cast<float>(std::sin(0.1)), 1e-4);
}

TEST(DeskewTest, AppliesTheVelocityTranslation) {
    const double t_ref = 1785000000.0;
    auto data = MakeCloud({{0.0, 0.0, 0.0, t_ref + 0.1}});
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    const ImuHistory imu = ConstantYawRate(t_ref - 0.5, t_ref + 0.5, 0.0);

    DeskewOptions opts;
    opts.use_linear_velocity = true;
    const auto stats = DeskewCloudInPlace(data.data(), data.size(), M20_POINT_STEP, 1, layout, imu, t_ref, t_ref,
                                          Eigen::Vector3d(2.0, 0.0, 0.0), opts);

    EXPECT_TRUE(stats.deskewed);
    EXPECT_NEAR(PointX(data, 0), 0.2f, 1e-5);  // 2 m/s for 0.1 s
}

TEST(DeskewTest, BucketsRotationsByTheTimeQuantum) {
    const double t_ref = 1785000000.0;
    auto data = MakeCloud({{1.0, 0.0, 0.0, t_ref}, {1.0, 0.0, 0.0, t_ref + 0.02}, {1.0, 0.0, 0.0, t_ref + 0.04}});
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    const ImuHistory imu = ConstantYawRate(t_ref - 0.5, t_ref + 0.5, 1.0);

    DeskewOptions opts;
    opts.use_linear_velocity = false;
    opts.time_quantum_sec = 1.0;  // everything lands in one bucket
    const auto stats = DeskewCloudInPlace(data.data(), data.size(), M20_POINT_STEP, 3, layout, imu, t_ref, t_ref,
                                          Eigen::Vector3d::Zero(), opts);

    EXPECT_TRUE(stats.deskewed);
    EXPECT_EQ(stats.unique_rotations, 1u);
    // Absolute epoch times are near 1.8e9, where a double resolves about 0.24 us
    EXPECT_NEAR(stats.span_sec, 0.04, 1e-6);
    // One bucket means one rotation for all three points
    EXPECT_NEAR(PointX(data, 0), PointX(data, 2), 1e-6);
}

TEST(DeskewTest, LeavesTheCloudAloneWithoutImuHistory) {
    const double t_ref = 1785000000.0;
    auto data = MakeCloud({{1.0, 2.0, 3.0, t_ref + 0.1}});
    const auto original = data;
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    const ImuHistory imu{2.0};

    const auto stats = DeskewCloudInPlace(data.data(), data.size(), M20_POINT_STEP, 1, layout, imu, t_ref, t_ref,
                                          Eigen::Vector3d::Zero(), DeskewOptions());

    EXPECT_FALSE(stats.deskewed);
    EXPECT_EQ(data, original);
}

TEST(DeskewTest, RefusesATruncatedBuffer) {
    const double t_ref = 1785000000.0;
    auto data = MakeCloud({{1.0, 0.0, 0.0, t_ref}, {1.0, 0.0, 0.0, t_ref + 0.1}});
    const auto original = data;
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    const ImuHistory imu = ConstantYawRate(t_ref - 0.5, t_ref + 0.5, 1.0);

    // Claim three points in a buffer that holds two
    const auto stats = DeskewCloudInPlace(data.data(), data.size(), M20_POINT_STEP, 3, layout, imu, t_ref, t_ref,
                                          Eigen::Vector3d::Zero(), DeskewOptions());

    EXPECT_FALSE(stats.deskewed);
    EXPECT_EQ(data, original);
}

TEST(DeskewTest, PassesNonFinitePointsThrough) {
    const double t_ref = 1785000000.0;
    auto data = MakeCloud({{std::nan(""), 0.0, 0.0, t_ref + 0.1}, {1.0, 0.0, 0.0, t_ref + 0.1}});
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    const ImuHistory imu = ConstantYawRate(t_ref - 0.5, t_ref + 0.5, 1.0);

    DeskewOptions opts;
    opts.use_linear_velocity = false;
    const auto stats = DeskewCloudInPlace(data.data(), data.size(), M20_POINT_STEP, 2, layout, imu, t_ref, t_ref,
                                          Eigen::Vector3d::Zero(), opts);

    EXPECT_TRUE(stats.deskewed);
    EXPECT_EQ(stats.points, 1u);  // only the finite one was rewritten
    EXPECT_TRUE(std::isnan(PointX(data, 0)));
    EXPECT_NEAR(PointX(data, 1), static_cast<float>(std::cos(0.1)), 1e-4);
}

TEST(DeskewTest, ReferenceTimeAheadOfTheScanShiftsPointsBackwards) {
    // The reference is the newest odometry stamp, which is normally *after* the scan stamp
    const double t_scan = 1785000000.0;
    const double t_ref = t_scan + 0.05;
    auto data = MakeCloud({{1.0, 0.0, 0.0, t_scan}});
    const CloudLayout layout = CloudLayout::FromFields(M20Fields(), M20_POINT_STEP);
    const ImuHistory imu = ConstantYawRate(t_scan - 0.5, t_scan + 0.5, 1.0);

    DeskewOptions opts;
    opts.use_linear_velocity = false;
    const auto stats = DeskewCloudInPlace(data.data(), data.size(), M20_POINT_STEP, 1, layout, imu, t_scan, t_ref,
                                          Eigen::Vector3d::Zero(), opts);

    EXPECT_TRUE(stats.deskewed);
    // The point predates the reference by 0.05 s, so it rotates by -0.05 rad
    EXPECT_NEAR(PointY(data, 0), static_cast<float>(std::sin(-0.05)), 1e-4);
}
