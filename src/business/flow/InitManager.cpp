/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "business/flow/InitManager.h"

#include <ros/ros.h>
#include <turtlesim/Color.h>

/**
 * @ingroup twist
 *
 * Namespace for all TWIST modules
 */
namespace twist {


InitManager::InitManager(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle) {
    ROS_INFO("Initializing Init Manager");
}

}  // namespace twist
