/**
 * @file
 * @brief M20 localization interface — the OEM-facing side of the driver
 *
 * The Deep Robotics M20 navigation stack expects three topics that share one timestamp, with /ODOM
 * leading:
 *
 *   /ODOM             nav_msgs/Odometry, 10 Hz, frame "map", empty child frame, reliable.
 *                     Published the moment the fusion output arrives — never delayed by anything
 *                     below. In SLAM mode it comes from elsewhere and is only followed.
 *   /LOC_BODY_POINTS  sensor_msgs/PointCloud2 in base_link, motion-compensated to the newest /ODOM
 *                     stamp, so its header stamp is that of a real /ODOM message. Runs in a worker
 *                     thread (latest scan wins) so a slow scan never stalls the odometry.
 *   /LOCATION_STATUS  drdds/LocationStatus at 2 Hz, sharing the same stamp, reporting input liveness
 *                     and execution health.
 *
 * The module also feeds the robot's leg odometry (drdds/MotionInfo) back to the positioning sensor
 * as wheelspeed aiding, which is what the separate Python bridge used to do.
 *
 * Threading: /ODOM is published from the driver's parser thread, the cloud from the deskew worker,
 * the status from either the worker or the node's timer. Shared state is guarded as noted below.
 */

#ifndef __FIXPOSITION_DRIVER_M20_M20_LOCALIZATION_HPP__
#define __FIXPOSITION_DRIVER_M20_M20_LOCALIZATION_HPP__

/* LIBC/STL */
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

/* EXTERNAL */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <drdds/msg/location_status.hpp>
#include <drdds/msg/motion_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

/* PACKAGE */
#include "fixposition_driver_m20/deskew.hpp"
#include "fixposition_driver_m20/m20_params.hpp"
#include "fixposition_driver_m20/odometry_source_policy.hpp"
#include "fixposition_driver_m20/status_monitor.hpp"
#include "fixposition_driver_m20/time_utils.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief A pose and velocity handed to the module by the driver
 *
 * Deliberately free of both ROS and Fixposition SDK types: the driver node converts its own odometry
 * data into this, which keeps this package independent of the sensor's message zoo.
 */
struct OdomSample {
    bool valid = false;
    Stamp stamp;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();   //!< in the body frame
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();  //!< in the body frame
    std::array<double, 36> pose_cov = {};
    std::array<double, 36> twist_cov = {};

    bool AllFinite() const;
};

/**
 * @brief Wheelspeed to forward to the positioning sensor, in the sensor's own units (mm/s)
 */
struct WheelSpeedSample {
    bool vx_valid = false;
    int32_t vx = 0;
    bool vy_valid = false;
    int32_t vy = 0;
    bool vz_valid = false;
    int32_t vz = 0;
};

//! How the module hands wheelspeed to the driver
using WheelSpeedSink = std::function<void(const WheelSpeedSample&)>;

/**
 * @brief Declare and load the m20.* parameters
 *
 * @returns false if a parameter is present but unusable; the module is then not started
 */
bool LoadM20ParamsFromRos2(std::shared_ptr<rclcpp::Node>& nh, M20Params& params);

/**
 * @brief How many times this process has already re-executed itself (from the environment)
 */
int ReinitAttempt();

/**
 * @brief Replace this process with a fresh copy of itself, keeping the PID and the argument vector
 *
 * Used to recover from a DDS participant that came up deaf at boot. Returns only if the exec failed.
 *
 * @param[in] argv       the original argument vector, NULL-terminated
 * @param[in] delay_sec  how long to wait before re-executing
 */
void ReexecSelf(char** argv, double delay_sec);

/**
 * @brief The M20 localization module
 */
class M20Localization {
   public:
    /**
     * @param[in] nh               node handle
     * @param[in] params           configuration
     * @param[in] wheelspeed_sink  called with leg odometry converted to wheelspeed; may be empty
     */
    M20Localization(std::shared_ptr<rclcpp::Node> nh, const M20Params& params, WheelSpeedSink wheelspeed_sink);
    ~M20Localization();

    M20Localization(const M20Localization&) = delete;
    M20Localization& operator=(const M20Localization&) = delete;

    /**
     * @brief Advertise, subscribe and start the deskew worker
     */
    bool Start();

    /**
     * @brief Stop the worker and drop all endpoints
     */
    void Stop();

    /**
     * @brief Feed one odometry sample in
     *
     * DecideOdomInput() (odometry_source_policy.hpp) decides what becomes of it. A sample from the
     * mode's selected source is published (as /ODOM in positioning-only mode, on the handler alias
     * in both), becomes the pose that stamps the other two topics and keeps the status alive. In
     * SLAM mode the sensor's ENU output is not that source: it only marks the RTK input as arriving.
     *
     * @param[in] sample  the pose and velocity
     * @param[in] feed    where it came from: the driver's parser thread passes SENSOR_ENU for every
     *                    ENU odometry message, the module's own subscription passes SUBSCRIBED_ODOM
     */
    void OnOdometry(const OdomSample& sample, OdomFeed feed);

    /**
     * @brief Has the startup watchdog concluded that this process needs to be re-created?
     */
    bool ReinitRequested() const { return reinit_requested_.load(); }

   private:
    // Callbacks
    void CloudCb(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void ImuCb(const sensor_msgs::msg::Imu& msg);
    void MotionInfoCb(const drdds::msg::MotionInfo& msg);
    void OdomSubCb(const nav_msgs::msg::Odometry& msg);
    void StatusTimerCb();
    void ReinitCheckCb();

    // Deskew worker
    void CloudWorker();
    void ProcessCloud(sensor_msgs::msg::PointCloud2::UniquePtr msg);
    //! Recompute the cached cloud layout if the fields changed
    void UpdateLayout(const sensor_msgs::msg::PointCloud2& msg);

    void PublishStatus(const Stamp& stamp);
    //! Seconds on the steady clock since this module was created
    double SteadyNow() const;
    //! The node's notion of the current epoch time [s]
    double EpochNow() const;

    std::shared_ptr<rclcpp::Node> nh_;
    M20Params params_;
    WheelSpeedSink wheelspeed_sink_;
    rclcpp::Logger logger_;

    // Publishers / subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr handler_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr handler_cloud_pub_;
    rclcpp::Publisher<drdds::msg::LocationStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<drdds::msg::MotionInfo>::SharedPtr motion_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr matching_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr reinit_timer_;

    // Health
    StatusMonitor status_;

    // Newest pose. Lock: state_mutex_ (leaf — never taken while holding another lock)
    mutable std::mutex state_mutex_;
    Stamp newest_stamp_;
    Eigen::Vector3d newest_velocity_ = Eigen::Vector3d::Zero();
    bool have_newest_ = false;

    // IMU window. Lock: imu_mutex_ (leaf)
    std::mutex imu_mutex_;
    ImuHistory imu_history_;

    // Scan hand-off to the worker. Lock: cloud_mutex_ (leaf)
    std::mutex cloud_mutex_;
    std::condition_variable cloud_cv_;
    sensor_msgs::msg::PointCloud2::UniquePtr pending_cloud_;
    bool stop_worker_ = false;
    uint64_t dropped_clouds_ = 0;
    std::thread worker_;

    // Stamp retiming (parser thread only)
    HostClockRetimer retimer_;

    // Extrinsic, empty when identity
    bool have_extrinsic_ = false;
    Eigen::Matrix3d extrinsic_rot_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d extrinsic_trans_ = Eigen::Vector3d::Zero();

    // Cloud layout cache (worker thread only)
    std::vector<PointFieldDesc> cached_fields_;
    uint32_t cached_point_step_ = 0;
    CloudLayout cached_layout_;

    // Status scheduling. Lock: status_mutex_ (leaf) — the worker publishes on the schedule while the
    // node's timer covers for it when the lidar goes quiet
    std::mutex status_mutex_;
    double status_period_ = 0.5;
    bool have_status_schedule_ = false;
    double next_status_due_ = 0.0;   //!< steady time of the next cloud-synced status publish
    double last_status_sent_ = 0.0;  //!< steady time of the last cloud-synced status publish
    std::atomic<uint64_t> status_seq_;

    // Wheelspeed rate cap (subscription thread only)
    double last_wheelspeed_sent_ = 0.0;

    // Counters and startup watchdog
    std::atomic<uint64_t> odom_count_;
    uint64_t cloud_count_ = 0;
    std::chrono::steady_clock::time_point start_time_;
    int reinit_attempt_ = 0;
    std::atomic<bool> reinit_requested_;
};

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_M20_LOCALIZATION_HPP__
