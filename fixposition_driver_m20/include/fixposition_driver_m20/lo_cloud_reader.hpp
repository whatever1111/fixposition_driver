/**
 *  @file
 *  @brief PointCloud2 reader on a dedicated loopback-only DDS participant
 *
 * The M20's OEM lidar driver publishes /LIDAR/POINTS from a DDS participant whose UDP transport is
 * bound to 127.0.0.1 only. Fast DDS 2.6 (as shipped with ROS 2 Humble) no longer announces a
 * loopback locator for a participant that also has real network interfaces, and the OEM side's
 * whitelist cannot answer to anything but loopback — so the node's regular (rmw-managed,
 * multi-interface) participant can never complete discovery with that writer, no matter how its
 * profile is arranged. Humble's rmw offers no per-subscription participant selection either.
 *
 * This class therefore bypasses rmw for this one input: it owns a bare Fast DDS DomainParticipant
 * whose only transport is UDPv4 on 127.0.0.1, with a single best-effort DataReader on the ROS topic,
 * and hands every sample to a callback as the regular ROS message type. Everything else in the node
 * (all publishers, all other inputs) stays on the rmw participant.
 */

#ifndef __FIXPOSITION_DRIVER_M20_LO_CLOUD_READER_HPP__
#define __FIXPOSITION_DRIVER_M20_LO_CLOUD_READER_HPP__

/* LIBC/STL */
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

/* EXTERNAL */
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief Best-effort PointCloud2 subscription on a loopback-only DDS participant
 *
 * The callback runs on a Fast DDS listener thread, so it must be thread-safe against the node's
 * executor callbacks (M20Localization::CloudCb is: the status monitor is atomic and the cloud
 * handoff is mutex-guarded).
 */
class LoCloudReader {
   public:
    using CloudCallback = std::function<void(sensor_msgs::msg::PointCloud2::UniquePtr)>;

    /**
     * @param[in] ros_topic  ROS topic name (e.g. "/LIDAR/POINTS")
     * @param[in] domain_id  DDS domain, from the node's context
     * @param[in] depth      KEEP_LAST history depth
     * @param[in] callback   receives every sample; called from a DDS listener thread
     */
    LoCloudReader(const std::string& ros_topic, uint32_t domain_id, int depth, CloudCallback callback);
    ~LoCloudReader();

    LoCloudReader(const LoCloudReader&) = delete;
    LoCloudReader& operator=(const LoCloudReader&) = delete;

    /**
     * @brief Did participant, topic and reader all come up?
     */
    bool ok() const;

    /**
     * @brief ROS topic name -> DDS topic name ("/LIDAR/POINTS" -> "rt/LIDAR/POINTS")
     */
    static std::string DdsTopicName(const std::string& ros_topic);

   private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_LO_CLOUD_READER_HPP__
