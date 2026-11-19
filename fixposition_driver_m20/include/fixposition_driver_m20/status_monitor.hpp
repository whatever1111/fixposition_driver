/**
 * @file
 * @brief Localization health tracking behind drdds/LocationStatus — ROS-free
 *
 * Inputs are tracked by liveness (last time something arrived) and failures by a sticky hold, so a
 * single hiccup stays visible for a few seconds instead of flickering past the 2 Hz status rate.
 * The state machine is a straight port of the Python bridge that has been running on the robot. Its
 * result is a verdict and a state per field, which no wire contract has numbered yet: the contract
 * the robot reads turns them into values (status_contract.hpp).
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

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief Tracked inputs
 *
 * RTK says that odometry is arriving, not that GNSS is healthy. Every sample accepted from the
 * selected odometry source marks it, and so does every valid, finite ENU output of the positioning
 * sensor while another node's /ODOM is followed. input_val.rtk therefore reads "the followed /ODOM
 * or the sensor's ENU output is arriving", and stays set without any GNSS for as long as either one
 * keeps delivering samples that pass those checks.
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
 * @brief The overall verdict. The first that applies wins, in this order.
 */
enum class StatusVerdict {
    UNINITIALIZED,  //!< no pose from the selected odometry source yet
    LOST,           //!< the odometry stopped, or a non-finite value arrived: stop navigating
    LOW_QUALITY,    //!< an input is missing or a failure is being held: keep watching
    NORMAL,
};

/**
 * @brief What an input_val field says about its input
 *
 * The lidar, the IMU, the leg odometry and the RTK input are monitored in every configuration. The
 * tag and the reflective strip are not fitted on the M20, and nothing consumes them.
 */
enum class InputState {
    NOT_USED,  //!< this configuration does not consume the input at all; never degrades the verdict
    FRESH,     //!< received within its timeout
    MISSING,   //!< monitored, but silent past its timeout or never received
};

/**
 * @brief One evaluation of the health state, field by field as in drdds/LocationStatusValue
 */
struct StatusHealth {
    StatusVerdict verdict = StatusVerdict::UNINITIALIZED;
    // exec_val
    bool overtime = false;         //!< a scan overran its budget within the error hold
    bool numeric_error = false;    //!< a non-finite value arrived within the error hold
    bool timestamp_error = false;  //!< the clocks disagreed or a stamp jumped within the error hold
    // loss_val
    bool odom_stalled = true;     //!< the selected odometry is not flowing
    bool leg_odom_error = true;   //!< the leg odometry is not arriving
    bool matching_error = false;  //!< monitored scan matching is not arriving
    bool pub_error = false;       //!< a publication failed within the error hold
    // input_val
    InputState lidar = InputState::MISSING;
    InputState imu = InputState::MISSING;
    InputState leg_odom = InputState::MISSING;
    InputState rtk = InputState::MISSING;
    InputState tag = InputState::NOT_USED;
    InputState reflective_strip = InputState::NOT_USED;
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
    StatusHealth Compute(double now) const;

   private:
    bool ErrorActive(StatusError error, double now) const;
    double Timeout(StatusInput input) const;
    //! The state of an input this configuration monitors
    InputState Monitored(StatusInput input, double now) const;

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
