/**
 * @file
 * @brief Implementation of the lidar deskewing numerics (see deskew.hpp)
 */

/* LIBC/STL */
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

/* PACKAGE */
#include "fixposition_driver_m20/deskew.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

namespace {

//! Per-point time field names seen in the wild, in preference order
const char* const TIME_FIELD_CANDIDATES[] = {"timestamp", "time", "t", "time_stamp", "stamp"};

//! Points sampled to decide whether times are absolute or scan-relative
constexpr size_t TIME_SAMPLE_COUNT = 64;

//! Upper bound on the rotation lookup table; the bucket width is widened rather than exceeding it
constexpr size_t MAX_ROTATION_TABLE_ENTRIES = 4096;

//! A scan stamp and an absolute point time may differ by at most this much [s]
constexpr double ABSOLUTE_TIME_TOLERANCE = 5.0;

//! Size in bytes of a point field datatype, 0 if unsupported
size_t FieldTypeSize(const uint8_t datatype) {
    switch (datatype) {
        case POINT_FIELD_INT8:
        case POINT_FIELD_UINT8:
            return 1;
        case POINT_FIELD_INT16:
        case POINT_FIELD_UINT16:
            return 2;
        case POINT_FIELD_INT32:
        case POINT_FIELD_UINT32:
        case POINT_FIELD_FLOAT32:
            return 4;
        case POINT_FIELD_FLOAT64:
            return 8;
        default:
            return 0;
    }
}

//! Can this datatype carry a per-point time?
bool IsUsableTimeType(const uint8_t datatype) {
    return (datatype == POINT_FIELD_FLOAT64) || (datatype == POINT_FIELD_FLOAT32) ||
           (datatype == POINT_FIELD_UINT32) || (datatype == POINT_FIELD_INT32);
}

//! Read a float32 at a byte offset (memcpy: the field may be unaligned, e.g. point_step 26)
float ReadF32(const uint8_t* p) {
    float v = 0.0f;
    std::memcpy(&v, p, sizeof(v));
    return v;
}

void WriteF32(uint8_t* p, const float v) { std::memcpy(p, &v, sizeof(v)); }

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------

CloudLayout CloudLayout::FromFields(const std::vector<PointFieldDesc>& fields, const uint32_t point_step) {
    CloudLayout layout;

    bool have_x = false;
    bool have_y = false;
    bool have_z = false;
    for (const auto& field : fields) {
        // x/y/z must be float32: deskewing writes the coordinates back in place
        if ((field.datatype != POINT_FIELD_FLOAT32) || (field.count != 1) ||
            ((field.offset + sizeof(float)) > point_step)) {
            continue;
        }
        if (field.name == "x") {
            layout.x_offset = field.offset;
            have_x = true;
        } else if (field.name == "y") {
            layout.y_offset = field.offset;
            have_y = true;
        } else if (field.name == "z") {
            layout.z_offset = field.offset;
            have_z = true;
        }
    }
    layout.valid = have_x && have_y && have_z;

    for (const char* const candidate : TIME_FIELD_CANDIDATES) {
        for (const auto& field : fields) {
            if ((field.name != candidate) || (field.count != 1) || !IsUsableTimeType(field.datatype) ||
                ((field.offset + FieldTypeSize(field.datatype)) > point_step)) {
                continue;
            }
            layout.time_offset = field.offset;
            layout.time_datatype = field.datatype;
            layout.time_field_name = field.name;
            layout.has_time = true;
            break;
        }
        if (layout.has_time) {
            break;
        }
    }

    return layout;
}

// ---------------------------------------------------------------------------------------------------------------------

double ReadPointTime(const uint8_t* point, const CloudLayout& layout) {
    const uint8_t* p = point + layout.time_offset;
    switch (layout.time_datatype) {
        case POINT_FIELD_FLOAT64: {
            double v = 0.0;
            std::memcpy(&v, p, sizeof(v));
            return v;
        }
        case POINT_FIELD_FLOAT32: {
            return static_cast<double>(ReadF32(p));
        }
        // Integer time fields carry nanoseconds by convention
        case POINT_FIELD_UINT32: {
            uint32_t v = 0;
            std::memcpy(&v, p, sizeof(v));
            return static_cast<double>(v) * 1e-9;
        }
        case POINT_FIELD_INT32: {
            int32_t v = 0;
            std::memcpy(&v, p, sizeof(v));
            return static_cast<double>(v) * 1e-9;
        }
        default:
            return std::numeric_limits<double>::quiet_NaN();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

PointTimeMode DetectPointTimeMode(const uint8_t* data, const uint32_t point_step, const size_t n_points,
                                  const CloudLayout& layout, const double t_scan) {
    if ((data == nullptr) || (n_points == 0) || (point_step == 0) || !layout.has_time) {
        return PointTimeMode::UNKNOWN;
    }

    const size_t stride = std::max<size_t>(1, n_points / TIME_SAMPLE_COUNT);
    std::vector<double> samples;
    samples.reserve(TIME_SAMPLE_COUNT + 1);
    for (size_t i = 0; i < n_points; i += stride) {
        const double t = ReadPointTime(data + (i * point_step), layout);
        if (std::isfinite(t)) {
            samples.push_back(t);
        }
    }
    if (samples.empty()) {
        return PointTimeMode::UNKNOWN;
    }

    std::nth_element(samples.begin(), samples.begin() + (samples.size() / 2), samples.end());
    const double median = samples[samples.size() / 2];

    if (std::fabs(median - t_scan) < ABSOLUTE_TIME_TOLERANCE) {
        return PointTimeMode::ABSOLUTE;
    }
    if ((median >= 0.0) && (median < ABSOLUTE_TIME_TOLERANCE)) {
        return PointTimeMode::RELATIVE;
    }
    return PointTimeMode::UNKNOWN;
}

// ---------------------------------------------------------------------------------------------------------------------

ImuHistory::ImuHistory(const double keep_sec) : keep_sec_{keep_sec} {}

void ImuHistory::Add(const double t, const Eigen::Vector3d& gyro) {
    if (!std::isfinite(t) || !gyro.allFinite()) {
        return;
    }
    // Samples must be monotonic for the running integral to mean anything
    if (!samples_.empty() && (t <= samples_.back().t)) {
        return;
    }

    Sample sample;
    sample.t = t;
    sample.w = gyro;
    if (samples_.empty()) {
        sample.cum.setZero();
    } else {
        const Sample& prev = samples_.back();
        sample.cum = prev.cum + (0.5 * (prev.w + gyro) * (t - prev.t));
    }
    samples_.push_back(sample);

    // Drop what has aged out, but never the last sample
    const double cutoff = t - keep_sec_;
    while ((samples_.size() > 1) && (samples_.front().t < cutoff)) {
        samples_.pop_front();
    }
}

void ImuHistory::Clear() { samples_.clear(); }

double ImuHistory::OldestTime() const { return samples_.empty() ? 0.0 : samples_.front().t; }

double ImuHistory::NewestTime() const { return samples_.empty() ? 0.0 : samples_.back().t; }

Eigen::Vector3d ImuHistory::ThetaAt(const double t) const {
    if (samples_.empty()) {
        return Eigen::Vector3d::Zero();
    }
    if (t <= samples_.front().t) {
        return samples_.front().cum;
    }
    if (t >= samples_.back().t) {
        return samples_.back().cum;
    }

    const auto it = std::lower_bound(samples_.begin(), samples_.end(), t,
                                     [](const Sample& sample, const double value) { return sample.t < value; });
    const auto prev = it - 1;
    const double span = it->t - prev->t;
    if (!(span > 0.0)) {
        return it->cum;
    }
    const double alpha = (t - prev->t) / span;
    return prev->cum + (alpha * (it->cum - prev->cum));
}

Eigen::Vector3d ImuHistory::RotVec(const double t_from, const double t_to) const {
    if (samples_.empty() || !std::isfinite(t_from) || !std::isfinite(t_to)) {
        return Eigen::Vector3d::Zero();
    }
    return ThetaAt(t_to) - ThetaAt(t_from);
}

// ---------------------------------------------------------------------------------------------------------------------

Eigen::Matrix3d RotVecToMatrix(const Eigen::Vector3d& rotvec) {
    if (!rotvec.allFinite()) {
        return Eigen::Matrix3d::Identity();
    }
    const double angle = rotvec.norm();
    // Negated comparison so that a NaN angle also lands on identity
    if (!(angle > 1e-12)) {
        return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(angle, rotvec / angle).toRotationMatrix();
}

// ---------------------------------------------------------------------------------------------------------------------

DeskewStats DeskewCloudInPlace(uint8_t* data, const size_t size_bytes, const uint32_t point_step,
                               const size_t n_points, const CloudLayout& layout, const ImuHistory& imu,
                               const double t_scan, const double t_ref, const Eigen::Vector3d& vel_body,
                               const DeskewOptions& opts) {
    DeskewStats stats;

    if ((data == nullptr) || (n_points == 0) || (point_step == 0) || !layout.valid || !layout.has_time ||
        imu.Empty() || !std::isfinite(t_scan) || !std::isfinite(t_ref)) {
        return stats;
    }
    // Refuse a buffer that does not hold the points it claims rather than reading past its end
    if (size_bytes < (n_points * static_cast<size_t>(point_step))) {
        return stats;
    }

    const PointTimeMode mode = DetectPointTimeMode(data, point_step, n_points, layout, t_scan);
    if (mode == PointTimeMode::UNKNOWN) {
        return stats;
    }
    // Point times are turned into seconds relative to the reference time
    const double time_bias = (mode == PointTimeMode::ABSOLUTE) ? -t_ref : (t_scan - t_ref);

    // Pass 1: the range of relative times present in this cloud
    double rel_min = std::numeric_limits<double>::infinity();
    double rel_max = -std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < n_points; i++) {
        const double t = ReadPointTime(data + (i * point_step), layout);
        if (!std::isfinite(t)) {
            continue;
        }
        const double rel = t + time_bias;
        rel_min = std::min(rel_min, rel);
        rel_max = std::max(rel_max, rel);
    }
    if (!std::isfinite(rel_min) || !std::isfinite(rel_max)) {
        return stats;
    }
    stats.span_sec = rel_max - rel_min;

    // Rotation lookup table over quantised relative times. Widen the bucket rather than let a long
    // or corrupt scan blow the table up.
    double quantum = opts.time_quantum_sec;
    if ((quantum > 0.0) && (((rel_max - rel_min) / quantum) + 1.0 > static_cast<double>(MAX_ROTATION_TABLE_ENTRIES))) {
        quantum = (rel_max - rel_min) / static_cast<double>(MAX_ROTATION_TABLE_ENTRIES - 1);
    }

    std::vector<Eigen::Matrix3d> rotations;
    int64_t first_bucket = 0;
    int64_t last_bucket = 0;
    if (quantum > 0.0) {
        first_bucket = std::llround(rel_min / quantum);
        last_bucket = std::llround(rel_max / quantum);
        const size_t n_buckets = static_cast<size_t>(last_bucket - first_bucket) + 1u;
        rotations.resize(n_buckets);
        for (size_t i = 0; i < n_buckets; i++) {
            const double rel = static_cast<double>(first_bucket + static_cast<int64_t>(i)) * quantum;
            rotations[i] = RotVecToMatrix(imu.RotVec(t_ref, t_ref + rel));
        }
    }
    stats.unique_rotations = rotations.empty() ? n_points : rotations.size();

    // Pass 2: rewrite every point into the body pose at t_ref
    for (size_t i = 0; i < n_points; i++) {
        uint8_t* point = data + (i * point_step);
        const Eigen::Vector3d in(static_cast<double>(ReadF32(point + layout.x_offset)),
                                 static_cast<double>(ReadF32(point + layout.y_offset)),
                                 static_cast<double>(ReadF32(point + layout.z_offset)));
        const double t = ReadPointTime(point, layout);
        if (!in.allFinite() || !std::isfinite(t)) {
            continue;
        }
        const double rel = t + time_bias;

        Eigen::Vector3d body = in;
        if (opts.extrinsic_rot != nullptr) {
            body = (*opts.extrinsic_rot) * body;
        }
        if (opts.extrinsic_trans != nullptr) {
            body += *opts.extrinsic_trans;
        }

        Eigen::Vector3d out;
        if (rotations.empty()) {
            out = RotVecToMatrix(imu.RotVec(t_ref, t_ref + rel)) * body;
        } else {
            int64_t bucket = std::llround(rel / quantum);
            bucket = std::min(std::max(bucket, first_bucket), last_bucket);
            out = rotations[static_cast<size_t>(bucket - first_bucket)] * body;
        }
        if (opts.use_linear_velocity) {
            out += vel_body * rel;
        }

        WriteF32(point + layout.x_offset, static_cast<float>(out.x()));
        WriteF32(point + layout.y_offset, static_cast<float>(out.y()));
        WriteF32(point + layout.z_offset, static_cast<float>(out.z()));
        stats.points++;
    }

    stats.deskewed = true;
    return stats;
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
