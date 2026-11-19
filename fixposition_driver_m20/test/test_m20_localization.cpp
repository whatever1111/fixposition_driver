/**
 * @file
 * @brief Tests for the module's ROS side: how it reads its parameters
 */

/* LIBC/STL */
#include <memory>
#include <string>
#include <vector>

/* EXTERNAL */
#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

/* PACKAGE */
#include "fixposition_driver_m20/m20_localization.hpp"

namespace {

using fixposition::m20::LoadM20ParamsFromRos2;
using fixposition::m20::M20Params;
using fixposition::m20::StatusContract;

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

TEST(M20ParamsTest, StatusContractDefaultsToLegacy) {
    M20Params params;
    ASSERT_TRUE(Load({}, params));
    EXPECT_EQ(params.status_contract, StatusContract::LEGACY);

    // The default is declared, so it shows up in introspection
    std::shared_ptr<rclcpp::Node> node = MakeNode({});
    ASSERT_TRUE(LoadM20ParamsFromRos2(node, params));
    EXPECT_EQ(node->get_parameter("m20.status_contract").as_string(), "legacy");
}

TEST(M20ParamsTest, StatusContractTakesEitherName) {
    M20Params v120;
    ASSERT_TRUE(Load({rclcpp::Parameter("m20.status_contract", "v1.2.0")}, v120));
    EXPECT_EQ(v120.status_contract, StatusContract::V1_2_0);

    M20Params legacy;
    ASSERT_TRUE(Load({rclcpp::Parameter("m20.status_contract", "legacy")}, legacy));
    EXPECT_EQ(legacy.status_contract, StatusContract::LEGACY);
}

TEST(M20ParamsTest, UnknownStatusContractFailsTheLoad) {
    for (const char* text : {"", "v1.2", "V1.2.0", "1.2.0", "v1.3.0", "LEGACY"}) {
        M20Params params;
        EXPECT_FALSE(Load({rclcpp::Parameter("m20.status_contract", text)}, params)) << '"' << text << '"';
    }
    // Whether or not the module is enabled: a value the driver cannot honour is never silently replaced
    M20Params disabled;
    EXPECT_FALSE(
        Load({rclcpp::Parameter("m20.enable", false), rclcpp::Parameter("m20.status_contract", "v2")}, disabled));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
