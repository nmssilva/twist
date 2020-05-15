/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "business/flow/InitManager.h"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#include "common/Logging.h"

namespace twist::business {

InitManager::InitManager(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle) {
    LOG_INFO("Initializing Init Manager");

    init();
}

void InitManager::init() {
    if (!navigationManager) {
        navigationManager = std::make_shared<NavigationManager>(nodeHandle);
    }
}

}  // namespace twist::business
