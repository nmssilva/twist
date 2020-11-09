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
    common::Configs::parseConfigFile() ? init() : initFailure();
}

void InitManager::init() {
    LOG_INFO("Initializing Init Manager");
    if (!navigationManager) {
        navigationManager = std::make_shared<NavigationManager>(nodeHandle);
    }
}

void InitManager::initFailure() {
    LOG_ERROR("Failed to parse configs from file" << std::string(CONFIG_FILE));
}

}  // namespace twist::business
