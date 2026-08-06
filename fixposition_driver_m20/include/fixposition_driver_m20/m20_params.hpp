/**
 * @file
 * @brief Parameters of the M20 localization module — ROS-free
 *
 * Everything lives under the "m20." parameter prefix so that it cannot collide with the driver's own
 * parameters. The module is off unless m20.enable is set, which keeps the driver a plain Fixposition
 * driver everywhere else.
 */

#ifndef __FIXPOSITION_DRIVER_M20_M20_PARAMS_HPP__
#define __FIXPOSITION_DRIVER_M20_M20_PARAMS_HPP__

/* LIBC/STL */
#include <string>
#include <vector>

/* PACKAGE */
#include "fixposition_driver_m20/status_monitor.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief Where the pose that stamps everything comes from
 */
enum class OdomSource {
    FPA_ODOMENU,  //!< the sensor's own ENU fusion output, republished as /ODOM (positioning-only mode)
    SUBSCRIBE,    //!< /ODOM is published by someone else (SLAM mode); only follow it
};

/**
 * @brief Configuration of the M20 localization module
 */
struct M20Params {
    //! Master switch. Off means the driver behaves exactly as upstream.
    bool enable = false;

    // ---- /ODOM ----------------------------------------------------------------------------------
    OdomSource odom_source = OdomSource::FPA_ODOMENU;
    std::string odom_topic = "/ODOM";
    //! The OEM's own /ODOM carries frame "map" and an EMPTY child frame; keep it that way
    std::string odom_frame_id = "map";
    std::string odom_child_frame_id = "";
    int odom_depth = 10;
    bool odom_reliable = true;
    //! The OEM strips the covariance to zero and downstream expects that
    bool zero_covariance = true;
    /**
     * Retime sensor stamps onto the host clock.
     *
     * The positioning sensor's clock can drift or jump against the robot's, while the lidar stamps —
     * and therefore the shared-stamp contract — live on the host's axis. The correction is a sliding
     * median of (host now - sensor stamp); a residual jump beyond retime_jump_sec resets it. With the
     * robot's PTP chain healthy this is a no-op, and it earns its keep when GNSS time is lost.
     */
    bool retime_to_host_clock = true;
    double retime_jump_sec = 0.5;

    // ---- /LOC_BODY_POINTS -----------------------------------------------------------------------
    std::string lidar_topic = "/LIDAR/POINTS";
    /**
     * Subscribe the lidar topic on a dedicated loopback-only DDS participant instead of the node's
     * own. Needed on the M20: its OEM lidar driver publishes from a 127.0.0.1-bound participant that
     * a Fast DDS 2.6 participant with real network interfaces can never discover (2.6 stops
     * announcing a loopback locator alongside real NICs, and the OEM whitelist cannot answer to
     * anything else). See lo_cloud_reader.hpp.
     */
    bool lidar_lo_participant = false;
    std::string imu_topic = "/IMU";
    std::string loc_body_points_topic = "/LOC_BODY_POINTS";
    std::string loc_body_frame_id = "base_link";
    int loc_body_depth = 5;
    bool loc_body_reliable = false;
    bool deskew_enable = true;
    bool deskew_use_linear_velocity = true;
    double deskew_time_quantum_sec = 0.001;
    //! lidar -> base_link extrinsic (x y z roll pitch yaw); identity on the M20, whose URDF TFs are all identity
    std::vector<double> lidar_to_base_xyzrpy = std::vector<double>(6, 0.0);

    // ---- /LOCATION_STATUS -----------------------------------------------------------------------
    std::string location_status_topic = "/LOCATION_STATUS";
    //! OEM contract: 2 Hz
    double location_status_rate_hz = 2.0;
    //! Scan-matching odometry to monitor in SLAM mode; empty disables the matching health check
    std::string matching_odom_topic = "";

    // ---- leg odometry input ---------------------------------------------------------------------
    std::string motion_info_topic = "/MOTION_INFO";
    //! Forward the leg odometry body velocity to the sensor as wheelspeed aiding
    bool wheelspeed_enable = true;
    //! Wheelspeed is sent in mm/s, leg odometry is in m/s
    double wheelspeed_scale_factor = 1000.0;
    bool wheelspeed_use_y = false;
    //! Cap the forwarding rate; the sensor needs no more, and /MOTION_INFO can run at hundreds of Hz
    double motion_info_max_rate_hz = 50.0;

    // ---- health ---------------------------------------------------------------------------------
    StatusTimeouts timeouts;
    /**
     * Above the 0.1 s scan period, so benign jitter does not flag the status; a sustained overrun
     * still trips it on every cloud (latest-scan-wins keeps the pipeline live, at a reduced rate).
     */
    double overtime_threshold_sec = 0.15;
    //! Disagreement between the odometry and lidar clocks that counts as a timestamp error [s]
    double timestamp_skew_threshold_sec = 1.0;

    // ---- startup self-heal ----------------------------------------------------------------------
    /**
     * A DDS participant created at boot has twice been seen coming up deaf to same-host publishers:
     * no lidar, no IMU, no leg odometry, no matter how long it waits. If nothing at all arrives
     * within this window the process re-executes itself, which is the in-process equivalent of the
     * manual service restart that reliably fixes it. 0 disables.
     */
    double boot_reinit_grace_sec = 45.0;
    int boot_reinit_max = 5;
};

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_M20_PARAMS_HPP__
