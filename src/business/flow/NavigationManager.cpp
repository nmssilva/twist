/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include <thread>

#include <geometry_msgs/PoseStamped.h>

#include "business/flow/InitManager.h"
#include "common/Configs.h"
#include "common/Logging.h"

namespace twist::business {

NavigationManager::NavigationManager(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle)
    , moveGoalPublisher(nodeHandle.advertise<geometry_msgs::PoseStamped>(MoveGoalTopic, 1000))
    , getTargetSubscriber(nodeHandle.subscribe(TargetTopic, 1000, &NavigationManager::setGoalFromTargetCallback, this)) {
    LOG_INFO("Initializing Navigation Manager");
}

void NavigationManager::setGoalFromTargetCallback(const std_msgs::String::ConstPtr& target) {
    LOG_INFO("Setting Goal from target: " << target->data);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "map";

    try {
        const auto locations{common::Configs::getValues().locations.at(static_cast<uint32_t>(common::locationConversion(target->data)))};

        poseStamped.pose.position.x = locations.x;
        poseStamped.pose.position.y = locations.y;
        poseStamped.pose.orientation.z = locations.z;
        poseStamped.pose.orientation.w = locations.w;
    } catch (const std::exception& e) {
        LOG_ERROR("Unknown location for " << target->data <<" with error:" << e.what());
    }

    moveGoalPublisher.publish(poseStamped);
}

}  // namespace twist::business
