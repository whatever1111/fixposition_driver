/**
 *  @file
 *  @brief PointCloud2 reader on a dedicated loopback-only DDS participant (implementation)
 *
 * Background in the header. The moving parts, mirroring what rmw_fastrtps does for a subscription,
 * minus everything we do not need:
 *  - a DomainParticipant whose only transport is UDPv4 whitelisted to 127.0.0.1 (matching the OEM
 *    profile's buffer sizes so ~1 MB fragmented clouds fit),
 *  - a TopicDataType that deserializes the wire CDR into sensor_msgs::msg::PointCloud2 through the
 *    message's own rosidl fastrtps typesupport,
 *  - a best-effort, volatile, KEEP_LAST reader with the realloc memory policy (the payloads are
 *    unbounded, preallocated-only would truncate them).
 */

/* LIBC/STL */
#include <utility>

/* EXTERNAL */
#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_fastrtps_cpp.hpp>

/* PACKAGE */
#include "fixposition_driver_m20/lo_cloud_reader.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

namespace {

namespace ddscpp = eprosima::fastdds::dds;
namespace ddsrtps = eprosima::fastrtps::rtps;

// The OEM profile's transport parameters; the clouds are ~1 MB fragmented at maxMessageSize.
constexpr uint32_t kSocketBufferSize = 8388608;
constexpr uint32_t kMaxMessageSize = 65500;

/**
 * @brief sensor_msgs::msg::PointCloud2 as a Fast DDS TopicDataType
 *
 * Serialization goes through the message's generated fastrtps typesupport, exactly as rmw does, so
 * the wire format is identical to any rclcpp publisher/subscription of the same type.
 */
class PointCloud2DataType : public ddscpp::TopicDataType {
   public:
    PointCloud2DataType() {
        setName("sensor_msgs::msg::dds_::PointCloud2_");
        // Unbounded type: this is only the pool's initial allocation, the realloc memory policy on
        // the reader grows it per sample
        m_typeSize = 4u + 1024u;
        m_isGetKeyDefined = false;
    }

    bool serialize(void* data, ddsrtps::SerializedPayload_t* payload) override {
        const auto* msg = static_cast<const sensor_msgs::msg::PointCloud2*>(data);
        eprosima::fastcdr::FastBuffer buffer(reinterpret_cast<char*>(payload->data), payload->max_size);
        eprosima::fastcdr::Cdr ser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
        ser.serialize_encapsulation();
        if (!sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(*msg, ser)) {
            return false;
        }
        payload->encapsulation = (ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS) ? CDR_BE : CDR_LE;
        payload->length = static_cast<uint32_t>(ser.getSerializedDataLength());
        return true;
    }

    bool deserialize(ddsrtps::SerializedPayload_t* payload, void* data) override {
        auto* msg = static_cast<sensor_msgs::msg::PointCloud2*>(data);
        eprosima::fastcdr::FastBuffer buffer(reinterpret_cast<char*>(payload->data), payload->length);
        eprosima::fastcdr::Cdr deser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);
        deser.read_encapsulation();
        return sensor_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(deser, *msg);
    }

    std::function<uint32_t()> getSerializedSizeProvider(void* data) override {
        return [data]() -> uint32_t {
            const auto* msg = static_cast<const sensor_msgs::msg::PointCloud2*>(data);
            return 4u + static_cast<uint32_t>(sensor_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(*msg, 0));
        };
    }

    // createData/deleteData are the Fast DDS ownership interface — raw new/delete is its contract
    void* createData() override { return new sensor_msgs::msg::PointCloud2(); }
    void deleteData(void* data) override { delete static_cast<sensor_msgs::msg::PointCloud2*>(data); }

    bool getKey(void* /*data*/, ddsrtps::InstanceHandle_t* /*handle*/, bool /*force_md5*/) override { return false; }
};

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------

class LoCloudReader::Impl : public ddscpp::DataReaderListener {
   public:
    Impl(const std::string& ros_topic, uint32_t domain_id, int depth, CloudCallback callback)
        : callback_(std::move(callback)) {
        // Participant: loopback only. This is the entire point — announcing any real interface makes
        // the OEM's loopback-confined participant unable to answer (see header).
        ddscpp::DomainParticipantQos pqos = ddscpp::PARTICIPANT_QOS_DEFAULT;
        pqos.name("fixposition_m20_lo_cloud_reader");
        auto udp = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
        udp->interfaceWhiteList.push_back("127.0.0.1");
        udp->sendBufferSize = kSocketBufferSize;
        udp->receiveBufferSize = kSocketBufferSize;
        udp->maxMessageSize = kMaxMessageSize;
        udp->non_blocking_send = true;
        pqos.transport().user_transports.push_back(udp);
        pqos.transport().use_builtin_transports = false;

        auto* factory = ddscpp::DomainParticipantFactory::get_instance();
        participant_ = factory->create_participant(domain_id, pqos);
        if (participant_ == nullptr) {
            return;
        }

        type_ = ddscpp::TypeSupport(new PointCloud2DataType());
        if (type_.register_type(participant_) != eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
            return;
        }

        topic_ = participant_->create_topic(DdsTopicName(ros_topic), type_.get_type_name(), ddscpp::TOPIC_QOS_DEFAULT);
        if (topic_ == nullptr) {
            return;
        }

        subscriber_ = participant_->create_subscriber(ddscpp::SUBSCRIBER_QOS_DEFAULT);
        if (subscriber_ == nullptr) {
            return;
        }

        // Same semantics as the node's sensor-input QoS: newest-only, matches both reliable and
        // best-effort writers
        ddscpp::DataReaderQos rqos = ddscpp::DATAREADER_QOS_DEFAULT;
        rqos.reliability().kind = ddscpp::BEST_EFFORT_RELIABILITY_QOS;
        rqos.durability().kind = ddscpp::VOLATILE_DURABILITY_QOS;
        rqos.history().kind = ddscpp::KEEP_LAST_HISTORY_QOS;
        rqos.history().depth = depth;
        // Unbounded payloads: without realloc the pool would be stuck at m_typeSize
        rqos.endpoint().history_memory_policy = ddsrtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

        reader_ = subscriber_->create_datareader(topic_, rqos, this, ddscpp::StatusMask::data_available());
    }

    ~Impl() override {
        if (participant_ != nullptr) {
            participant_->delete_contained_entities();
            ddscpp::DomainParticipantFactory::get_instance()->delete_participant(participant_);
        }
    }

    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    bool ok() const { return reader_ != nullptr; }

    void on_data_available(ddscpp::DataReader* reader) override {
        ddscpp::SampleInfo info;
        auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        while (reader->take_next_sample(msg.get(), &info) == eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
            if (!info.valid_data) {
                continue;
            }
            callback_(std::move(msg));
            msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        }
    }

   private:
    CloudCallback callback_;
    ddscpp::DomainParticipant* participant_ = nullptr;
    ddscpp::TypeSupport type_;
    ddscpp::Topic* topic_ = nullptr;
    ddscpp::Subscriber* subscriber_ = nullptr;
    ddscpp::DataReader* reader_ = nullptr;
};

// ---------------------------------------------------------------------------------------------------------------------

LoCloudReader::LoCloudReader(const std::string& ros_topic, uint32_t domain_id, int depth, CloudCallback callback)
    : impl_(std::make_unique<Impl>(ros_topic, domain_id, depth, std::move(callback))) {}

LoCloudReader::~LoCloudReader() = default;

bool LoCloudReader::ok() const { return impl_->ok(); }

std::string LoCloudReader::DdsTopicName(const std::string& ros_topic) {
    if (ros_topic.empty() || (ros_topic[0] != '/')) {
        return "rt/" + ros_topic;
    }
    return "rt" + ros_topic;
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
