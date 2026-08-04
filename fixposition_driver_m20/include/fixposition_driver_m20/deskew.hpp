/**
 * @file
 * @brief Motion compensation ("deskewing") of a lidar scan — ROS-free numerics
 *
 * Every point of a rolling-shutter lidar scan is captured at a slightly different time and hence
 * from a slightly different body pose. This module rewrites the points so that they all express the
 * body pose at one reference time:
 *
 *     p_ref = R(ref <- i) * p_i + v_body * (t_i - t_ref)
 *
 * The rotation comes from integrating the IMU body rates, the translation from the body velocity of
 * the newest odometry. Both terms are small (a scan spans ~0.1 s), so first-order accumulation is
 * far below lidar noise.
 *
 * Nothing here depends on ROS or on the Fixposition SDK, so it is unit-testable on its own.
 */

#ifndef __FIXPOSITION_DRIVER_M20_DESKEW_HPP__
#define __FIXPOSITION_DRIVER_M20_DESKEW_HPP__

/* LIBC/STL */
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

/* EXTERNAL */
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief Point field datatypes, mirroring the sensor_msgs/PointField constants
 *
 * Duplicated here (instead of including the message) to keep this header free of ROS. The values are
 * part of the message definition and therefore fixed.
 */
enum PointFieldType : uint8_t {
    POINT_FIELD_INT8 = 1,
    POINT_FIELD_UINT8 = 2,
    POINT_FIELD_INT16 = 3,
    POINT_FIELD_UINT16 = 4,
    POINT_FIELD_INT32 = 5,
    POINT_FIELD_UINT32 = 6,
    POINT_FIELD_FLOAT32 = 7,
    POINT_FIELD_FLOAT64 = 8,
};

/**
 * @brief One entry of a point cloud's field description (name, byte offset, datatype)
 */
struct PointFieldDesc {
    std::string name;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 1;
};

/**
 * @brief Where x/y/z and the per-point timestamp live inside one point
 */
struct CloudLayout {
    bool valid = false;      //!< x, y and z were found and are float32
    bool has_time = false;   //!< a per-point time field was found
    uint32_t x_offset = 0;   //!< byte offset of x within a point
    uint32_t y_offset = 0;   //!< byte offset of y within a point
    uint32_t z_offset = 0;   //!< byte offset of z within a point
    uint32_t time_offset = 0;  //!< byte offset of the time field within a point
    uint8_t time_datatype = 0;  //!< datatype of the time field
    std::string time_field_name;  //!< name of the field used as time

    /**
     * @brief Determine the layout from a cloud's field descriptions
     *
     * The time field is picked from a list of names used in the wild, in preference order. Points
     * whose x/y/z are not float32 are rejected (valid stays false) — deskewing writes back in place
     * and the M20 lidar, like every other lidar we handle, uses float32.
     *
     * @param[in] fields      field descriptions, as they appear in the message
     * @param[in] point_step  bytes per point, used to reject fields that do not fit
     */
    static CloudLayout FromFields(const std::vector<PointFieldDesc>& fields, uint32_t point_step);
};

/**
 * @brief How the per-point time values relate to the scan stamp
 */
enum class PointTimeMode {
    UNKNOWN,   //!< neither — do not deskew
    ABSOLUTE,  //!< absolute epoch seconds (the M20 lidar does this)
    RELATIVE,  //!< seconds since the start of the scan
};

/**
 * @brief Read one point's time value, whatever its datatype
 *
 * Integer time fields are interpreted as nanoseconds (the convention of the lidars that use them).
 *
 * @returns the time in seconds, or NaN if the datatype is not supported
 */
double ReadPointTime(const uint8_t* point, const CloudLayout& layout);

/**
 * @brief Decide whether the per-point times are absolute or relative to the scan
 *
 * Decided from the median of a subsample, so a few corrupt points cannot flip the verdict.
 *
 * @param[in] data        packed point data
 * @param[in] point_step  bytes per point
 * @param[in] n_points    number of points
 * @param[in] layout      cloud layout (must have a time field)
 * @param[in] t_scan      the scan's own stamp [s]
 */
PointTimeMode DetectPointTimeMode(const uint8_t* data, uint32_t point_step, size_t n_points,
                                  const CloudLayout& layout, double t_scan);

/**
 * @brief Sliding window of IMU angular rates, for integrating body rotation over a scan
 *
 * Not thread-safe: the caller serialises access (the ROS layer copies the window under a lock before
 * handing it to the deskew worker).
 */
class ImuHistory {
   public:
    /**
     * @param[in] keep_sec  how much history to retain [s]
     */
    explicit ImuHistory(double keep_sec = 2.0);

    /**
     * @brief Append a sample; samples must arrive in non-decreasing time order
     *
     * Out-of-order samples are dropped. Samples older than the retention window are discarded.
     *
     * @param[in] t     sample time [s]
     * @param[in] gyro  body angular rate [rad/s]
     */
    void Add(double t, const Eigen::Vector3d& gyro);

    void Clear();
    bool Empty() const { return samples_.empty(); }
    size_t Size() const { return samples_.size(); }

    /**
     * @brief Oldest / newest sample time [s] (0.0 when empty)
     */
    double OldestTime() const;
    double NewestTime() const;

    /**
     * @brief Body rotation from @p t_from to @p t_to, as a rotation vector [rad]
     *
     * Trapezoidal integration of the body rates, first-order (commutative) accumulation — exact
     * enough for the ~0.1 s window of one scan. Outside the sample range the integral is clamped
     * (constant extrapolation), so a query slightly ahead of the newest sample degrades gracefully
     * instead of extrapolating noise.
     */
    Eigen::Vector3d RotVec(double t_from, double t_to) const;

   private:
    struct Sample {
        double t = 0.0;
        Eigen::Vector3d w = Eigen::Vector3d::Zero();
        Eigen::Vector3d cum = Eigen::Vector3d::Zero();  //!< integral of w up to t, arbitrary origin
    };

    /**
     * @brief Interpolated value of the running integral at @p t
     */
    Eigen::Vector3d ThetaAt(double t) const;

    std::deque<Sample> samples_;
    double keep_sec_;
};

/**
 * @brief Rotation matrix from a rotation vector (Rodrigues), safe at zero angle
 */
Eigen::Matrix3d RotVecToMatrix(const Eigen::Vector3d& rotvec);

/**
 * @brief Knobs for DeskewCloudInPlace()
 */
struct DeskewOptions {
    //! Apply the body velocity translation term
    bool use_linear_velocity = true;
    /**
     * Bucket width for the rotation lookup [s]; 0 computes a rotation per distinct point time.
     *
     * A lidar with near-per-point stamps would otherwise need thousands of Rodrigues evaluations per
     * cloud, while 1 ms of rotation at even 2 rad/s is ~0.1° — far below the sensor's own noise. The
     * exact point time is still used for the translation term.
     */
    double time_quantum_sec = 0.001;
    //! Optional lidar -> body extrinsic, applied before deskewing. Null means identity.
    const Eigen::Matrix3d* extrinsic_rot = nullptr;
    const Eigen::Vector3d* extrinsic_trans = nullptr;
};

/**
 * @brief What DeskewCloudInPlace() did
 */
struct DeskewStats {
    bool deskewed = false;        //!< false means the cloud was left untouched
    size_t points = 0;            //!< points transformed
    size_t unique_rotations = 0;  //!< distinct rotations evaluated (rotation table size)
    double span_sec = 0.0;        //!< newest minus oldest point time in the cloud
};

/**
 * @brief Deskew a packed point cloud in place
 *
 * Leaves the cloud untouched (returning deskewed = false) whenever the inputs do not allow an honest
 * transform: unusable layout, unrecognised time values, or no IMU history. The caller then publishes
 * the raw scan rather than a wrong one.
 *
 * Points with non-finite coordinates or times are passed through unchanged.
 *
 * @param[in,out] data        packed point data, modified in place
 * @param[in]     size_bytes  size of @p data, used to bound the writes
 * @param[in]     point_step  bytes per point
 * @param[in]     n_points    number of points
 * @param[in]     layout      cloud layout, from CloudLayout::FromFields()
 * @param[in]     imu         IMU window covering the scan
 * @param[in]     t_scan      the scan's own stamp [s]
 * @param[in]     t_ref       reference time to express all points at [s]
 * @param[in]     vel_body    body velocity [m/s], used when opts.use_linear_velocity
 * @param[in]     opts        options
 */
DeskewStats DeskewCloudInPlace(uint8_t* data, size_t size_bytes, uint32_t point_step, size_t n_points,
                               const CloudLayout& layout, const ImuHistory& imu, double t_scan, double t_ref,
                               const Eigen::Vector3d& vel_body, const DeskewOptions& opts);

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_DESKEW_HPP__
