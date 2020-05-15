/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include <thread>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "business/flow/InitManager.h"
#include "common/Mapping.h"
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
    poseStamped.pose.position.x = LocationMap.at(locationConversion(target->data)).first;
    poseStamped.pose.position.y = LocationMap.at(locationConversion(target->data)).second;
    poseStamped.pose.orientation.w = 1;

    moveGoalPublisher.publish(poseStamped);
}

std::map<Location, std::pair<x, y>>& NavigationManager::getLocationMap() {
    return LocationMap;
}

}  // namespace twist::business
