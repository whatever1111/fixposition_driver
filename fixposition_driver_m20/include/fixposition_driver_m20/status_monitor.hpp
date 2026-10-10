/**
 * @file
 * @brief Localization health tracking behind drdds/LocationStatus — ROS-free
 *
 * Inputs are tracked by liveness (last time something arrived) and failures by a sticky hold, so a
 * single hiccup stays visible for a few seconds instead of flickering past the 2 Hz status rate.
 * The state machine is a straight port of the Python bridge that has been running on the robot, and
 * its verdicts follow the OEM's total_status contract:
 *
 *   0 uninitialized  needs a localization init / map
 *   1 normal
 *   2 low quality    degraded inputs or execution errors, keep watching
 *   3 lost           stop navigating and re-initialize
 *
 * All times are seconds from a steady clock; the caller passes "now" in, which keeps the class pure
 * and testable.
 */

#ifndef __FIXPOSITION_DRIVER_M20_STATUS_MONITOR_HPP__
#define __FIXPOSITION_DRIVER_M20_STATUS_MONITOR_HPP__

/* LIBC/STL */
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

//! drdds LocationStatusValue.total_status, per the OEM specification
constexpr uint8_t TOTAL_UNINITIALIZED = 0;
constexpr uint8_t TOTAL_NORMAL = 1;
constexpr uint8_t TOTAL_LOW_QUALITY = 2;
constexpr uint8_t TOTAL_LOST = 3;

/**
 * Sub-field flag values. The OEM has not published the semantics of the exec_val / loss_val /
 * input_val flags; these are the values the field-validated Python bridge used. Change them here if
 * and when the specification arrives.
 */
constexpr uint8_t ERR_OK = 0;
constexpr uint8_t ERR_ERROR = 1;
constexpr uint8_t INPUT_OK = 1;
constexpr uint8_t INPUT_MISSING = 0;

/**
 * @brief Tracked inputs
 *
 * RTK says that odometry is arriving, not that GNSS is healthy. Every sample accepted from the
 * selected odometry source marks it, and so does every valid ENU output of the positioning sensor
 * while another node's /ODOM is followed. input_val.rtk therefore reads "the followed /ODOM or the
 * sensor's ENU output is arriving", and stays set without any GNSS for as long as either one flows.
 */
enum class StatusInput {
    LIDAR = 0,  //!< lidar point clouds
    IMU,        //!< body IMU
    LEG_ODOM,   //!< leg odometry (drdds MotionInfo)
    RTK,        //!< the selected odometry source or the positioning sensor's ENU output (see above)
    MATCHING,   //!< scan-matching odometry, only in SLAM mode
    COUNT
};

/**
 * @brief Tracked failure events
 */
enum class StatusError {
    OVERTIME = 0,  //!< a scan took longer to process than its budget
    NUMERIC,       //!< a non-finite value arrived
    TIMESTAMP,     //!< clocks disagree, or a stamp jumped
    PUBLISH,       //!< publishing threw
    COUNT
};

/**
 * @brief How long an input may be silent before it counts as missing [s]
 */
struct StatusTimeouts {
    double lidar = 0.5;
    double imu = 0.3;
    double leg_odom = 0.5;
    double rtk = 1.0;
    double matching = 0.5;
    //! How long a failure event keeps being reported after it last happened [s]
    double error_hold = 5.0;
};

/**
 * @brief One evaluation of the health state, mapping 1:1 onto drdds/LocationStatusValue
 */
struct StatusSnapshot {
    uint8_t total_status = TOTAL_UNINITIALIZED;
    // exec_val
    uint8_t overtime = ERR_OK;
    uint8_t numeric_error = ERR_OK;
    uint8_t timestamp_error = ERR_OK;
    // loss_val
    uint8_t odom_error = ERR_OK;
    uint8_t leg_odom_error = ERR_OK;
    uint8_t matching_error = ERR_OK;
    uint8_t pub_error = ERR_OK;
    // input_val
    uint8_t lidar = INPUT_MISSING;
    uint8_t imu = INPUT_MISSING;
    uint8_t leg_odom = INPUT_MISSING;
    uint8_t rtk = INPUT_MISSING;
    uint8_t tag = INPUT_MISSING;
    uint8_t reflective_strip = INPUT_MISSING;
};

/**
 * @brief Liveness and failure bookkeeping behind /LOCATION_STATUS
 *
 * Thread-safe: marks come from the sensor callbacks, the driver's parser thread and the deskew
 * worker, while Compute() runs on whichever thread publishes the status.
 */
class StatusMonitor {
   public:
    /**
     * @param[in] timeouts          liveness windows
     * @param[in] matching_enabled  false in positioning-only mode, where there is no scan matching
     *                              and matching_error is therefore always OK
     */
    StatusMonitor(const StatusTimeouts& timeouts, bool matching_enabled);

    /**
     * @brief Note that an input has just been received
     */
    void MarkRx(StatusInput input, double now);

    /**
     * @brief Note that a failure has just happened
     */
    void MarkError(StatusError error, double now);

    /**
     * @brief Note that an odometry message has just gone out (or come in, in SLAM mode)
     */
    void MarkOdomAlive(double now);

    /**
     * @brief Note that a valid pose has been seen, which leaves the uninitialized state for good
     */
    void SetInitialized();

    bool Initialized() const;

    /**
     * @brief Has this input ever been received? (used by the startup deafness watchdog)
     */
    bool EverReceived(StatusInput input) const;

    /**
     * @brief Has this input been received recently enough to count as present?
     */
    bool Fresh(StatusInput input, double now) const;

    /**
     * @brief Evaluate the current health
     */
    StatusSnapshot Compute(double now) const;

   private:
    bool ErrorActive(StatusError error, double now) const;
    double Timeout(StatusInput input) const;

    StatusTimeouts timeouts_;
    bool matching_enabled_;
    std::array<std::atomic<double>, static_cast<std::size_t>(StatusInput::COUNT)> last_rx_;
    std::array<std::atomic<double>, static_cast<std::size_t>(StatusError::COUNT)> last_error_;
    std::atomic<double> last_odom_alive_;
    std::atomic<bool> initialized_;
};

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_STATUS_MONITOR_HPP__
