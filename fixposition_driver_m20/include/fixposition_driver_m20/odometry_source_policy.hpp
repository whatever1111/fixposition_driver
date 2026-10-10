/**
 * @file
 * @brief Which odometry feed the module follows, and what the other one may still do — ROS-free
 *
 * Two feeds can reach the module: the positioning sensor's ENU fusion output, handed over by the
 * driver's parser thread, and an /ODOM published by another node, which the module subscribes to in
 * SLAM mode. Each mode has exactly one selected source. Only the selected source is published,
 * stamps the cloud and the status, keeps the odometry alive and initializes the status. The sensor
 * keeps talking in SLAM mode too; there its output is no more than a sign that the sensor is alive.
 *
 * The decision is a pure function of the mode, the feed and the sample's validity, so the whole
 * matrix can be tested without a node.
 */

#ifndef __FIXPOSITION_DRIVER_M20_ODOMETRY_SOURCE_POLICY_HPP__
#define __FIXPOSITION_DRIVER_M20_ODOMETRY_SOURCE_POLICY_HPP__

/* PACKAGE */
#include "fixposition_driver_m20/m20_params.hpp"

namespace fixposition {
namespace m20 {
/* ****************************************************************************************************************** */

/**
 * @brief Where an odometry sample came from
 */
enum class OdomFeed {
    SENSOR_ENU,       //!< the positioning sensor's ENU fusion output (FP_A-ODOMENU), via the driver
    SUBSCRIBED_ODOM,  //!< an /ODOM published by another node, via the module's own subscription
};

/**
 * @brief What the module does with one odometry sample
 */
enum class OdomInputAction {
    IGNORE,                  //!< nothing at all
    RTK_HEALTH_ONLY,         //!< mark the RTK input as arriving, and nothing else
    SELECTED,                //!< the full path: publish, become the stamp reference, keep the status alive
    SELECTED_NUMERIC_ERROR,  //!< the selected source delivered a non-finite value: flag it and drop the sample
};

/**
 * @brief Decide what to do with one odometry sample
 *
 * There is no fallback from one feed to the other: a feed that is not the mode's selected source
 * never reaches the full path, whatever the state of the selected one.
 *
 * @param[in] mode    the configured odometry source
 * @param[in] feed    where the sample came from
 * @param[in] valid   the sample's producer marked it valid
 * @param[in] finite  every number in the sample is finite
 */
constexpr OdomInputAction DecideOdomInput(const OdomSource mode, const OdomFeed feed, const bool valid,
                                          const bool finite) {
    bool selected = false;
    switch (mode) {
        case OdomSource::FPA_ODOMENU:
            selected = (feed == OdomFeed::SENSOR_ENU);
            break;
        case OdomSource::SUBSCRIBE:
            selected = (feed == OdomFeed::SUBSCRIBED_ODOM);
            break;
    }
    if (selected) {
        if (!valid) {
            return OdomInputAction::IGNORE;
        }
        return finite ? OdomInputAction::SELECTED : OdomInputAction::SELECTED_NUMERIC_ERROR;
    }

    // While another node's /ODOM is followed the sensor is not the pose source, but a usable sample
    // still shows that it is talking. An unusable one shows nothing, and is not this module's error.
    if ((mode == OdomSource::SUBSCRIBE) && (feed == OdomFeed::SENSOR_ENU) && valid && finite) {
        return OdomInputAction::RTK_HEALTH_ONLY;
    }
    return OdomInputAction::IGNORE;
}

/* ****************************************************************************************************************** */
}  // namespace m20
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_M20_ODOMETRY_SOURCE_POLICY_HPP__
