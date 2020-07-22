/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "business/flow/InitManager.h"

#include <geometry_msgs/PoseStamped.h>

#include "common/Configs.h"
#include "common/Logging.h"

namespace twist::business {

InitManager::InitManager(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle) {
    if (common::Configs::parseConfigFile()) {
        LOG_INFO("Initializing Init Manager");

        init();
    } else {
        LOG_ERROR("Failed to parse configs");
    }
}

void InitManager::init() {
    if (!navigationManager) {
        navigationManager = std::make_shared<NavigationManager>(nodeHandle);
    }
}

}  // namespace twist::business
