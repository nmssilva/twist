/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "business/flow/InitManager.h"

#include <ros/ros.h>

namespace twist {
namespace business {

InitManager::InitManager(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle) {
    ROS_INFO("Initializing Init Manager");
}

}  // namespace business
}  // namespace twist
