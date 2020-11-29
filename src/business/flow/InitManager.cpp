/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "business/flow/InitManager.h"

#include <geometry_msgs/PoseStamped.h>
#include <sound_play/sound_play.h>

#include "common/Configs.h"
#include "common/Logging.h"

namespace twist::business {

InitManager::InitManager(ros::NodeHandle& nodeHandle)
    : nodeHandle(nodeHandle) {

    sound_play::SoundClient sc;
    ros::Duration(1, 0).sleep();
    //sc.playWaveFromPkg("sound_play", "sounds/BACKINGUP.ogg");
    sc.playWave( "/home/luisfilipedias/CTW/03_Fikalab/01_ROS/00_Wspc/src/twist/resources/audio/bug.wav");

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
