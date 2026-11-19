/**
 * @file
 * @brief Tests for the module's ROS side: how it reads its parameters and what it subscribes
 */

/* LIBC/STL */
#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

/* PACKAGE */
#include "fixposition_driver_m20/m20_localization.hpp"

namespace {

using fixposition::m20::LoadM20ParamsFromRos2;
using fixposition::m20::M20Localization;
using fixposition::m20::M20Params;

//! A fresh node carrying @p overrides, the way a params file would set them
std::shared_ptr<rclcpp::Node> MakeNode(const std::vector<rclcpp::Parameter>& overrides) {
    static int count = 0;
    return std::make_shared<rclcpp::Node>("m20_test_" + std::to_string(count++),
                                          rclcpp::NodeOptions().parameter_overrides(overrides));
}

//! Load the m20.* parameters from a node carrying @p overrides; returns what LoadM20ParamsFromRos2 did
bool Load(const std::vector<rclcpp::Parameter>& overrides, M20Params& params) {
    std::shared_ptr<rclcpp::Node> node = MakeNode(overrides);
    return LoadM20ParamsFromRos2(node, params);
}

}  // namespace

TEST(M20ParamsTest, InitialPoseDefaults) {
    std::shared_ptr<rclcpp::Node> node = MakeNode({});
    M20Params params;
    ASSERT_TRUE(LoadM20ParamsFromRos2(node, params));
    EXPECT_EQ(params.initialpose_topic, "/initialpose");
    EXPECT_EQ(params.timeouts.initialpose_settle, 2.0);
    // Declared, so they show up in introspection
    EXPECT_EQ(node->get_parameter("m20.initialpose_topic").as_string(), "/initialpose");
    EXPECT_EQ(node->get_parameter("m20.initialpose_settle_sec").as_double(), 2.0);
}

TEST(M20ParamsTest, InitialPoseSettleMustBeFiniteAndPositive) {
    M20Params params;
    ASSERT_TRUE(Load(
        {rclcpp::Parameter("m20.initialpose_topic", "/pose_in"), rclcpp::Parameter("m20.initialpose_settle_sec", 0.25)},
        params));
    EXPECT_EQ(params.initialpose_topic, "/pose_in");
    EXPECT_EQ(params.timeouts.initialpose_settle, 0.25);

    for (const double bad : {0.0, -0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        M20Params rejected;
        EXPECT_FALSE(Load({rclcpp::Parameter("m20.initialpose_settle_sec", bad)}, rejected)) << bad;
    }
}

namespace {

//! Does the graph show @p subscriber among the nodes subscribing @p topic? Other nodes on the
//! network may subscribe it too; they do not count.
bool Subscribes(const rclcpp::Node& subscriber, const std::string& topic) {
    const std::vector<rclcpp::TopicEndpointInfo> endpoints = subscriber.get_subscriptions_info_by_topic(topic);
    return std::any_of(endpoints.begin(), endpoints.end(), [&subscriber](const rclcpp::TopicEndpointInfo& endpoint) {
        return (endpoint.node_name() == subscriber.get_name()) &&
               (endpoint.node_namespace() == subscriber.get_namespace());
    });
}

//! A started module on a node of its own, set up by @p overrides; it stops when it goes out of scope
struct StartedModule {
    explicit StartedModule(std::vector<rclcpp::Parameter> overrides = {}) {
        overrides.emplace_back("m20.boot_reinit_grace_sec", 0.0);
        node = MakeNode(overrides);
        M20Params params;
        EXPECT_TRUE(LoadM20ParamsFromRos2(node, params));
        module = std::make_unique<M20Localization>(node, params, nullptr);
        EXPECT_TRUE(module->Start());
    }
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<M20Localization> module;
};

}  // namespace

TEST(M20LocalizationTest, SubscribesInitialPosesUnlessTheTopicIsEmpty) {
    // Started before the module that does subscribe, so that a subscription of its own would be in the
    // graph by the time that one's is
    const StartedModule disabled({rclcpp::Parameter("m20.initialpose_topic", "")});
    const StartedModule subscribing;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!Subscribes(*subscribing.node, "/initialpose") && (std::chrono::steady_clock::now() < deadline)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(Subscribes(*subscribing.node, "/initialpose"));
    EXPECT_FALSE(Subscribes(*disabled.node, "/initialpose"));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
