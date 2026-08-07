/**
 * @file
 * @brief Implementation of the M20 localization interface (see m20_localization.hpp)
 */

/* LIBC/STL */
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <utility>

/* EXTERNAL */
#include <unistd.h>

/* PACKAGE */
#include "fixposition_driver_m20/m20_localization.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

namespace {

//! Environment variable carrying the re-execution count across process images
const char* const REINIT_ATTEMPT_ENV = "FP_M20_REINIT_ATTEMPT";

//! How often the startup deafness watchdog looks [s]
constexpr double REINIT_CHECK_PERIOD = 5.0;

//! How much IMU history to keep for deskewing [s]
constexpr double IMU_HISTORY_SEC = 2.0;

//! Log one line every this many clouds
constexpr uint64_t CLOUD_LOG_PERIOD = 600;

//! Log one line every this many odometry messages
constexpr uint64_t ODOM_LOG_PERIOD = 200;

rclcpp::QoS MakeQos(const int depth, const bool reliable) {
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(std::max(1, depth)))};
    if (reliable) {
        qos.reliable();
    } else {
        qos.best_effort();
    }
    return qos;
}

//! Build the lidar -> body extrinsic; returns false when it is (or might as well be) identity
bool MakeExtrinsic(const std::vector<double>& xyzrpy, Eigen::Matrix3d& rot, Eigen::Vector3d& trans) {
    if (xyzrpy.size() != 6) {
        return false;
    }
    const bool all_zero = std::all_of(xyzrpy.begin(), xyzrpy.end(), [](double v) { return std::fabs(v) < 1e-12; });
    if (all_zero) {
        return false;
    }
    trans = Eigen::Vector3d(xyzrpy[0], xyzrpy[1], xyzrpy[2]);
    rot = (Eigen::AngleAxisd(xyzrpy[5], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(xyzrpy[4], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(xyzrpy[3], Eigen::Vector3d::UnitX()))
              .toRotationMatrix();
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------

bool OdomSample::AllFinite() const {
    return position.allFinite() && linear_velocity.allFinite() && angular_velocity.allFinite() &&
           std::isfinite(orientation.w()) && orientation.vec().allFinite();
}

// ---------------------------------------------------------------------------------------------------------------------

int ReinitAttempt() {
    const char* const value = std::getenv(REINIT_ATTEMPT_ENV);
    if (value == nullptr) {
        return 0;
    }
    const int attempt = std::atoi(value);
    return (attempt > 0) ? attempt : 0;
}

void ReexecSelf(char** argv, const double delay_sec) {
    const std::string next = std::to_string(ReinitAttempt() + 1);
    setenv(REINIT_ATTEMPT_ENV, next.c_str(), 1);
    if (delay_sec > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(delay_sec));
    }
    // Keeps the PID (so a launcher's pidfile stays valid) and rebuilds everything DDS from scratch
    execv("/proc/self/exe", argv);
    // Only reached if the exec failed
    if (argv != nullptr) {
        execvp(argv[0], argv);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool LoadM20ParamsFromRos2(std::shared_ptr<rclcpp::Node>& nh, M20Params& params) {
    auto logger = nh->get_logger();
    bool ok = true;

    // Small helper: declare with the struct's value as default, then read it back
    const auto load_bool = [&nh](const std::string& name, bool& value) {
        nh->declare_parameter(name, value);
        nh->get_parameter(name, value);
    };
    const auto load_double = [&nh](const std::string& name, double& value) {
        nh->declare_parameter(name, value);
        nh->get_parameter(name, value);
    };
    const auto load_int = [&nh](const std::string& name, int& value) {
        nh->declare_parameter(name, value);
        nh->get_parameter(name, value);
    };
    const auto load_string = [&nh](const std::string& name, std::string& value) {
        nh->declare_parameter(name, value);
        nh->get_parameter(name, value);
    };

    load_bool("m20.enable", params.enable);

    std::string odom_source = "fpa_odomenu";
    load_string("m20.odom_source", odom_source);
    if (odom_source == "fpa_odomenu") {
        params.odom_source = OdomSource::FPA_ODOMENU;
    } else if (odom_source == "subscribe") {
        params.odom_source = OdomSource::SUBSCRIBE;
    } else if (params.enable) {
        RCLCPP_ERROR(logger, "m20.odom_source must be \"fpa_odomenu\" or \"subscribe\", got \"%s\"",
                     odom_source.c_str());
        ok = false;
    }

    load_string("m20.odom_topic", params.odom_topic);
    load_string("m20.odom_frame_id", params.odom_frame_id);
    load_string("m20.odom_child_frame_id", params.odom_child_frame_id);
    load_int("m20.odom_depth", params.odom_depth);
    load_bool("m20.odom_reliable", params.odom_reliable);
    load_bool("m20.zero_covariance", params.zero_covariance);
    load_bool("m20.retime_to_host_clock", params.retime_to_host_clock);
    load_double("m20.retime_jump_sec", params.retime_jump_sec);

    load_string("m20.lidar_topic", params.lidar_topic);
    load_string("m20.imu_topic", params.imu_topic);
    load_string("m20.loc_body_points_topic", params.loc_body_points_topic);
    load_string("m20.loc_body_frame_id", params.loc_body_frame_id);
    load_int("m20.loc_body_depth", params.loc_body_depth);
    load_bool("m20.loc_body_reliable", params.loc_body_reliable);
    load_bool("m20.deskew_enable", params.deskew_enable);
    load_bool("m20.deskew_use_linear_velocity", params.deskew_use_linear_velocity);
    load_double("m20.deskew_time_quantum_sec", params.deskew_time_quantum_sec);

    nh->declare_parameter("m20.lidar_to_base_xyzrpy", params.lidar_to_base_xyzrpy);
    nh->get_parameter("m20.lidar_to_base_xyzrpy", params.lidar_to_base_xyzrpy);
    if (params.lidar_to_base_xyzrpy.size() != 6) {
        RCLCPP_ERROR(logger, "m20.lidar_to_base_xyzrpy must have exactly 6 elements (x y z roll pitch yaw), got %zu",
                     params.lidar_to_base_xyzrpy.size());
        ok = false;
    }

    load_string("m20.location_status_topic", params.location_status_topic);
    load_double("m20.location_status_rate_hz", params.location_status_rate_hz);
    if (!(params.location_status_rate_hz > 0.0)) {
        RCLCPP_ERROR(logger, "m20.location_status_rate_hz must be > 0, got %.3f", params.location_status_rate_hz);
        ok = false;
    }
    load_string("m20.matching_odom_topic", params.matching_odom_topic);

    load_string("m20.motion_info_topic", params.motion_info_topic);
    load_bool("m20.wheelspeed_enable", params.wheelspeed_enable);
    load_double("m20.wheelspeed_scale_factor", params.wheelspeed_scale_factor);
    load_bool("m20.wheelspeed_use_y", params.wheelspeed_use_y);
    load_double("m20.motion_info_max_rate_hz", params.motion_info_max_rate_hz);

    load_double("m20.timeouts.lidar_sec", params.timeouts.lidar);
    load_double("m20.timeouts.imu_sec", params.timeouts.imu);
    load_double("m20.timeouts.leg_odom_sec", params.timeouts.leg_odom);
    load_double("m20.timeouts.rtk_sec", params.timeouts.rtk);
    load_double("m20.timeouts.matching_sec", params.timeouts.matching);
    load_double("m20.timeouts.error_hold_sec", params.timeouts.error_hold);
    load_double("m20.overtime_threshold_sec", params.overtime_threshold_sec);
    load_double("m20.timestamp_skew_threshold_sec", params.timestamp_skew_threshold_sec);

    load_double("m20.boot_reinit_grace_sec", params.boot_reinit_grace_sec);
    load_int("m20.boot_reinit_max", params.boot_reinit_max);

    return ok;
}

// ---------------------------------------------------------------------------------------------------------------------

M20Localization::M20Localization(std::shared_ptr<rclcpp::Node> nh, const M20Params& params,
                                 WheelSpeedSink wheelspeed_sink)
    : nh_{std::move(nh)},
      params_{params},
      wheelspeed_sink_{std::move(wheelspeed_sink)},
      logger_{nh_->get_logger()},
      status_{params.timeouts, !params.matching_odom_topic.empty()},
      imu_history_{IMU_HISTORY_SEC},
      retimer_{params.retime_jump_sec},
      status_seq_{0},
      odom_count_{0},
      start_time_{std::chrono::steady_clock::now()},
      reinit_attempt_{ReinitAttempt()},
      reinit_requested_{false} {
    status_period_ = 1.0 / params_.location_status_rate_hz;
    have_extrinsic_ = MakeExtrinsic(params_.lidar_to_base_xyzrpy, extrinsic_rot_, extrinsic_trans_);
}

M20Localization::~M20Localization() { Stop(); }

double M20Localization::SteadyNow() const {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
}

double M20Localization::EpochNow() const { return static_cast<double>(nh_->now().nanoseconds()) * 1e-9; }

// ---------------------------------------------------------------------------------------------------------------------

bool M20Localization::Start() {
    const rclcpp::QoS odom_qos = MakeQos(params_.odom_depth, params_.odom_reliable);
    const rclcpp::QoS cloud_qos = MakeQos(params_.loc_body_depth, params_.loc_body_reliable);
    // Sensor inputs: shallow and best-effort — only the newest sample is worth anything, and this
    // matches both reliable and best-effort publishers
    const rclcpp::QoS input_qos = MakeQos(5, false);

    if (params_.odom_source == OdomSource::FPA_ODOMENU) {
        RCLCPP_INFO(logger_, "M20: publishing %s (frame \"%s\", child \"%s\") from the sensor's ENU fusion output",
                    params_.odom_topic.c_str(), params_.odom_frame_id.c_str(), params_.odom_child_frame_id.c_str());
        odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(params_.odom_topic, odom_qos);
    } else {
        RCLCPP_INFO(logger_, "M20: following %s for the pose and stamp (published elsewhere)",
                    params_.odom_topic.c_str());
        odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
            params_.odom_topic, input_qos, [this](const nav_msgs::msg::Odometry& msg) { OdomSubCb(msg); });
    }

    body_points_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(params_.loc_body_points_topic, cloud_qos);
    status_pub_ = nh_->create_publisher<drdds::msg::LocationStatus>(params_.location_status_topic, odom_qos);

    lidar_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
        params_.lidar_topic, input_qos,
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { CloudCb(std::move(msg)); });
    imu_sub_ = nh_->create_subscription<sensor_msgs::msg::Imu>(
        params_.imu_topic, input_qos, [this](const sensor_msgs::msg::Imu& msg) { ImuCb(msg); });
    motion_info_sub_ = nh_->create_subscription<drdds::msg::MotionInfo>(
        params_.motion_info_topic, input_qos, [this](const drdds::msg::MotionInfo& msg) { MotionInfoCb(msg); });
    if (!params_.matching_odom_topic.empty()) {
        matching_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
            params_.matching_odom_topic, input_qos,
            [this](const nav_msgs::msg::Odometry&) { status_.MarkRx(StatusInput::MATCHING, SteadyNow()); });
    }

    RCLCPP_INFO(logger_, "M20: deskewing %s -> %s (frame \"%s\", gyro %s, deskew %s, reference = newest %s stamp)",
                params_.lidar_topic.c_str(), params_.loc_body_points_topic.c_str(), params_.loc_body_frame_id.c_str(),
                params_.imu_topic.c_str(), params_.deskew_enable ? "on" : "off", params_.odom_topic.c_str());
    RCLCPP_INFO(logger_, "M20: publishing %s at %.2f Hz", params_.location_status_topic.c_str(),
                params_.location_status_rate_hz);

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        stop_worker_ = false;
    }
    worker_ = std::thread(&M20Localization::CloudWorker, this);

    status_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(status_period_),
                                           [this]() { StatusTimerCb(); });
    if (params_.boot_reinit_grace_sec > 0.0) {
        reinit_timer_ = nh_->create_wall_timer(std::chrono::duration<double>(REINIT_CHECK_PERIOD),
                                               [this]() { ReinitCheckCb(); });
    }
    if (reinit_attempt_ > 0) {
        RCLCPP_WARN(logger_, "M20: process was restarted by the deafness self-heal (attempt %d/%d)", reinit_attempt_,
                    params_.boot_reinit_max);
    }

    return true;
}

void M20Localization::Stop() {
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        stop_worker_ = true;
    }
    cloud_cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }

    status_timer_.reset();
    reinit_timer_.reset();
    lidar_sub_.reset();
    imu_sub_.reset();
    motion_info_sub_.reset();
    odom_sub_.reset();
    matching_sub_.reset();
    odom_pub_.reset();
    body_points_pub_.reset();
    status_pub_.reset();
}

// ---------------------------------------------------------------------------------------------------------------------

void M20Localization::OnOdometry(const OdomSample& sample) {
    const double steady = SteadyNow();
    if (!sample.valid) {
        return;
    }
    if (!sample.AllFinite()) {
        status_.MarkError(StatusError::NUMERIC, steady);
        RCLCPP_WARN_THROTTLE(logger_, *nh_->get_clock(), 5000, "M20: non-finite value in odometry — message dropped");
        return;
    }

    Stamp stamp = sample.stamp;
    // Only the sensor's own stamps need translating; a pose from another node is already on this axis
    if (params_.retime_to_host_clock && (params_.odom_source == OdomSource::FPA_ODOMENU)) {
        bool jumped = false;
        const double corrected = retimer_.Apply(stamp.ToSec(), EpochNow(), &jumped);
        if (jumped) {
            status_.MarkError(StatusError::TIMESTAMP, steady);
            RCLCPP_WARN(logger_, "M20: the sensor's clock stepped against the host — retiming re-converging");
        }
        stamp = Stamp::FromSec(corrected);
    }

    if (odom_pub_) {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp.sec = stamp.sec;
        msg.header.stamp.nanosec = stamp.nsec;
        msg.header.frame_id = params_.odom_frame_id;
        msg.child_frame_id = params_.odom_child_frame_id;
        msg.pose.pose.position.x = sample.position.x();
        msg.pose.pose.position.y = sample.position.y();
        msg.pose.pose.position.z = sample.position.z();
        msg.pose.pose.orientation.w = sample.orientation.w();
        msg.pose.pose.orientation.x = sample.orientation.x();
        msg.pose.pose.orientation.y = sample.orientation.y();
        msg.pose.pose.orientation.z = sample.orientation.z();
        msg.twist.twist.linear.x = sample.linear_velocity.x();
        msg.twist.twist.linear.y = sample.linear_velocity.y();
        msg.twist.twist.linear.z = sample.linear_velocity.z();
        msg.twist.twist.angular.x = sample.angular_velocity.x();
        msg.twist.twist.angular.y = sample.angular_velocity.y();
        msg.twist.twist.angular.z = sample.angular_velocity.z();
        if (!params_.zero_covariance) {
            std::copy(sample.pose_cov.begin(), sample.pose_cov.end(), msg.pose.covariance.begin());
            std::copy(sample.twist_cov.begin(), sample.twist_cov.end(), msg.twist.covariance.begin());
        }
        try {
            odom_pub_->publish(msg);
        } catch (const std::exception& ex) {
            status_.MarkError(StatusError::PUBLISH, steady);
            RCLCPP_ERROR_THROTTLE(logger_, *nh_->get_clock(), 5000, "M20: failed publishing odometry: %s", ex.what());
            return;
        }
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        newest_stamp_ = stamp;
        newest_velocity_ = sample.linear_velocity;
        have_newest_ = true;
    }
    status_.MarkRx(StatusInput::RTK, steady);
    status_.MarkOdomAlive(steady);
    status_.SetInitialized();

    const uint64_t count = odom_count_.fetch_add(1) + 1;
    const char* const verb = odom_pub_ ? "published" : "tracked";
    if (count == 1) {
        RCLCPP_INFO(logger_, "M20: first odometry message %s", verb);
    } else if ((count % ODOM_LOG_PERIOD) == 0) {
        RCLCPP_INFO(logger_, "M20: %" PRIu64 " odometry messages %s", count, verb);
    }
}

void M20Localization::OdomSubCb(const nav_msgs::msg::Odometry& msg) {
    OdomSample sample;
    sample.valid = true;
    sample.stamp.sec = msg.header.stamp.sec;
    sample.stamp.nsec = msg.header.stamp.nanosec;
    sample.position = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    sample.orientation = Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                            msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    sample.linear_velocity =
        Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    sample.angular_velocity =
        Eigen::Vector3d(msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z);
    OnOdometry(sample);
}

void M20Localization::ImuCb(const sensor_msgs::msg::Imu& msg) {
    const double steady = SteadyNow();
    status_.MarkRx(StatusInput::IMU, steady);

    const Eigen::Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    if (!gyro.allFinite()) {
        status_.MarkError(StatusError::NUMERIC, steady);
        return;
    }
    const double t = static_cast<double>(msg.header.stamp.sec) + (static_cast<double>(msg.header.stamp.nanosec) * 1e-9);

    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_history_.Add(t, gyro);
}

void M20Localization::MotionInfoCb(const drdds::msg::MotionInfo& msg) {
    const double steady = SteadyNow();
    status_.MarkRx(StatusInput::LEG_ODOM, steady);

    if (!params_.wheelspeed_enable || !wheelspeed_sink_) {
        return;
    }
    if (params_.motion_info_max_rate_hz > 0.0) {
        const double min_period = 1.0 / params_.motion_info_max_rate_hz;
        if ((steady - last_wheelspeed_sent_) < min_period) {
            return;
        }
        last_wheelspeed_sent_ = steady;
    }

    const double vel_x = static_cast<double>(msg.data.vel_x);
    const double vel_y = static_cast<double>(msg.data.vel_y);
    if (!std::isfinite(vel_x) || !std::isfinite(vel_y)) {
        status_.MarkError(StatusError::NUMERIC, steady);
        return;
    }

    WheelSpeedSample wheelspeed;
    wheelspeed.vx_valid = true;
    wheelspeed.vx = static_cast<int32_t>(std::lround(vel_x * params_.wheelspeed_scale_factor));
    wheelspeed.vy_valid = params_.wheelspeed_use_y;
    wheelspeed.vy = static_cast<int32_t>(std::lround(vel_y * params_.wheelspeed_scale_factor));
    wheelspeed_sink_(wheelspeed);
}

void M20Localization::CloudCb(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    const double steady = SteadyNow();
    status_.MarkRx(StatusInput::LIDAR, steady);

    const double t_scan =
        static_cast<double>(msg->header.stamp.sec) + (static_cast<double>(msg->header.stamp.nanosec) * 1e-9);
    if (std::fabs(EpochNow() - t_scan) > params_.timestamp_skew_threshold_sec) {
        status_.MarkError(StatusError::TIMESTAMP, steady);
    }

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (pending_cloud_) {
            // Latest scan wins: a stale scan helps nobody, and dropping it keeps the pipeline live
            dropped_clouds_++;
            RCLCPP_WARN_THROTTLE(logger_, *nh_->get_clock(), 10000,
                                 "M20: deskew is behind — dropped %" PRIu64 " older scan(s)", dropped_clouds_);
        }
        pending_cloud_ = std::move(msg);
    }
    cloud_cv_.notify_one();
}

// ---------------------------------------------------------------------------------------------------------------------

void M20Localization::CloudWorker() {
    while (true) {
        sensor_msgs::msg::PointCloud2::UniquePtr msg;
        {
            std::unique_lock<std::mutex> lock(cloud_mutex_);
            cloud_cv_.wait(lock, [this]() { return pending_cloud_ || stop_worker_; });
            if (stop_worker_) {
                return;
            }
            msg = std::move(pending_cloud_);
        }
        try {
            ProcessCloud(std::move(msg));
        } catch (const std::exception& ex) {
            // The worker must survive anything a single scan can do to it
            status_.MarkError(StatusError::PUBLISH, SteadyNow());
            RCLCPP_ERROR_THROTTLE(logger_, *nh_->get_clock(), 5000, "M20: cloud processing failed: %s", ex.what());
        }
    }
}

void M20Localization::UpdateLayout(const sensor_msgs::msg::PointCloud2& msg) {
    const bool same = (msg.point_step == cached_point_step_) && (msg.fields.size() == cached_fields_.size()) &&
                      std::equal(msg.fields.begin(), msg.fields.end(), cached_fields_.begin(),
                                 [](const sensor_msgs::msg::PointField& field, const PointFieldDesc& cached) {
                                     return (field.name == cached.name) && (field.offset == cached.offset) &&
                                            (field.datatype == cached.datatype) && (field.count == cached.count);
                                 });
    if (same) {
        return;
    }

    cached_fields_.clear();
    cached_fields_.reserve(msg.fields.size());
    for (const auto& field : msg.fields) {
        PointFieldDesc desc;
        desc.name = field.name;
        desc.offset = field.offset;
        desc.datatype = field.datatype;
        desc.count = field.count;
        cached_fields_.push_back(desc);
    }
    cached_point_step_ = msg.point_step;
    cached_layout_ = CloudLayout::FromFields(cached_fields_, cached_point_step_);

    if (cached_layout_.valid && cached_layout_.has_time) {
        RCLCPP_INFO(logger_, "M20: cloud layout — point_step %u, time field \"%s\"", cached_point_step_,
                    cached_layout_.time_field_name.c_str());
    } else {
        RCLCPP_WARN(logger_, "M20: cloud has no usable float32 x/y/z (%s) or per-point time field (%s) — relaying "
                             "without deskewing",
                    cached_layout_.valid ? "ok" : "missing", cached_layout_.has_time ? "ok" : "missing");
    }
}

void M20Localization::ProcessCloud(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    if (!msg) {
        return;
    }
    const auto started = std::chrono::steady_clock::now();
    const double t_scan =
        static_cast<double>(msg->header.stamp.sec) + (static_cast<double>(msg->header.stamp.nanosec) * 1e-9);

    // The cloud is stamped with the newest /ODOM stamp, which is what makes the three topics share a
    // timestamp. Before the first pose, or when the two clocks disagree wildly, keep the scan's own.
    Stamp ref;
    ref.sec = msg->header.stamp.sec;
    ref.nsec = msg->header.stamp.nanosec;
    double t_ref = t_scan;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    bool have_velocity = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (have_newest_) {
            const double newest_sec = newest_stamp_.ToSec();
            if (std::fabs(newest_sec - t_scan) <= params_.timestamp_skew_threshold_sec) {
                ref = newest_stamp_;
                t_ref = newest_sec;
                velocity = newest_velocity_;
                have_velocity = true;
            } else {
                status_.MarkError(StatusError::TIMESTAMP, SteadyNow());
            }
        }
    }

    DeskewStats stats;
    if (params_.deskew_enable) {
        UpdateLayout(*msg);
        if (cached_layout_.valid && cached_layout_.has_time) {
            ImuHistory imu_snapshot{IMU_HISTORY_SEC};
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                imu_snapshot = imu_history_;
            }
            DeskewOptions opts;
            opts.use_linear_velocity = params_.deskew_use_linear_velocity && have_velocity;
            opts.time_quantum_sec = params_.deskew_time_quantum_sec;
            if (have_extrinsic_) {
                opts.extrinsic_rot = &extrinsic_rot_;
                opts.extrinsic_trans = &extrinsic_trans_;
            }
            const size_t n_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
            stats = DeskewCloudInPlace(msg->data.data(), msg->data.size(), msg->point_step, n_points, cached_layout_,
                                       imu_snapshot, t_scan, t_ref, velocity, opts);
            if (!stats.deskewed) {
                RCLCPP_WARN_THROTTLE(logger_, *nh_->get_clock(), 10000,
                                     "M20: scan could not be deskewed (times or IMU history unusable) — relaying it "
                                     "unmodified");
            }
        }
    }

    msg->header.stamp.sec = ref.sec;
    msg->header.stamp.nanosec = ref.nsec;
    msg->header.frame_id = params_.loc_body_frame_id;

    const double process_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    try {
        body_points_pub_->publish(std::move(msg));
    } catch (const std::exception& ex) {
        status_.MarkError(StatusError::PUBLISH, SteadyNow());
        RCLCPP_ERROR_THROTTLE(logger_, *nh_->get_clock(), 5000, "M20: failed publishing body points: %s", ex.what());
    }
    const double total_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    if (total_sec > params_.overtime_threshold_sec) {
        status_.MarkError(StatusError::OVERTIME, SteadyNow());
    }

    // The status shares the cloud's stamp, published on each 1/rate boundary. The schedule advances
    // in fixed steps rather than from the last publish, which would quantise to the scan grid and
    // drift the rate low (the Python bridge averaged 1.8 Hz that way).
    {
        const double now = SteadyNow();
        bool due = false;
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            if (!have_status_schedule_) {
                have_status_schedule_ = true;
                next_status_due_ = now;
            }
            if (now >= next_status_due_) {
                due = true;
                last_status_sent_ = now;
                next_status_due_ += status_period_;
                if (now >= next_status_due_) {
                    // Fell behind, e.g. the lidar was silent for a while
                    next_status_due_ = now + status_period_;
                }
            }
        }
        if (due) {
            PublishStatus(ref);
        }
    }

    cloud_count_++;
    if (cloud_count_ == 1) {
        RCLCPP_INFO(logger_, "M20: first cloud published (%zu points, deskew %s, %.1f ms, stamp %.0f ms ahead of scan)",
                    stats.points, stats.deskewed ? "yes" : "NO", total_sec * 1e3, (t_ref - t_scan) * 1e3);
    } else if ((cloud_count_ % CLOUD_LOG_PERIOD) == 0) {
        RCLCPP_INFO(logger_, "M20: %" PRIu64 " clouds published (last %.1f ms, %zu points, %zu rotations)",
                    cloud_count_, total_sec * 1e3, stats.points, stats.unique_rotations);
    } else if (total_sec > params_.overtime_threshold_sec) {
        RCLCPP_WARN_THROTTLE(logger_, *nh_->get_clock(), 10000, "M20: slow cloud: %.1f ms (%.1f ms processing)",
                             total_sec * 1e3, process_sec * 1e3);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void M20Localization::StatusTimerCb() {
    // Only a fallback: keeps the status flowing while the lidar is silent, which is exactly when the
    // navigation stack most needs to be told about it
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (have_status_schedule_ && ((SteadyNow() - last_status_sent_) < (1.5 * status_period_))) {
            return;
        }
    }

    Stamp stamp;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (have_newest_) {
            stamp = newest_stamp_;
        } else {
            stamp = Stamp::FromSec(EpochNow());
        }
    }
    PublishStatus(stamp);
}

void M20Localization::PublishStatus(const Stamp& stamp) {
    if (!status_pub_) {
        return;
    }
    const StatusSnapshot snapshot = status_.Compute(SteadyNow());

    drdds::msg::LocationStatus msg;
    // Note the OEM's naming: LocationStatus carries "meta" where MotionInfo carries "header"
    msg.meta.frame_id = status_seq_.fetch_add(1) + 1;
    msg.meta.stamp.sec = stamp.sec;
    msg.meta.stamp.nanosec = stamp.nsec;

    msg.data.total_status = snapshot.total_status;
    msg.data.exec_val.overtime = snapshot.overtime;
    msg.data.exec_val.numeric_error = snapshot.numeric_error;
    msg.data.exec_val.timestamp_error = snapshot.timestamp_error;
    msg.data.loss_val.odom_error = snapshot.odom_error;
    msg.data.loss_val.leg_odom_error = snapshot.leg_odom_error;
    msg.data.loss_val.matching_error = snapshot.matching_error;
    msg.data.loss_val.pub_error = snapshot.pub_error;
    msg.data.input_val.lidar = snapshot.lidar;
    msg.data.input_val.imu = snapshot.imu;
    msg.data.input_val.leg_odom = snapshot.leg_odom;
    msg.data.input_val.rtk = snapshot.rtk;
    msg.data.input_val.tag = snapshot.tag;
    msg.data.input_val.reflective_strip = snapshot.reflective_strip;

    try {
        status_pub_->publish(msg);
    } catch (const std::exception& ex) {
        status_.MarkError(StatusError::PUBLISH, SteadyNow());
        RCLCPP_ERROR_THROTTLE(logger_, *nh_->get_clock(), 5000, "M20: failed publishing location status: %s",
                              ex.what());
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void M20Localization::ReinitCheckCb() {
    // Anything at all having arrived means the participant hears the robot, which is all this checks
    if (status_.EverReceived(StatusInput::LIDAR) || status_.EverReceived(StatusInput::IMU) ||
        status_.EverReceived(StatusInput::LEG_ODOM)) {
        reinit_timer_->cancel();
        return;
    }
    if (SteadyNow() <= params_.boot_reinit_grace_sec) {
        return;
    }
    if (reinit_attempt_ >= params_.boot_reinit_max) {
        RCLCPP_ERROR(logger_, "M20: still deaf to every input after %d process re-creations — giving up the self-heal; "
                              "check the sensor network and DDS configuration",
                     params_.boot_reinit_max);
        reinit_timer_->cancel();
        return;
    }

    RCLCPP_WARN(logger_, "M20: nothing received on %s, %s or %s within %.0f s of startup — re-creating the process to "
                         "get a fresh DDS participant (attempt %d/%d)",
                params_.lidar_topic.c_str(), params_.imu_topic.c_str(), params_.motion_info_topic.c_str(),
                params_.boot_reinit_grace_sec, reinit_attempt_ + 1, params_.boot_reinit_max);
    reinit_timer_->cancel();
    reinit_requested_.store(true);
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
