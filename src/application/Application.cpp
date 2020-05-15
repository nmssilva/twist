/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "Application.h"

#include <ros/node_handle.h>

namespace twist {

Application::Application(ros::NodeHandle& nodeHandle)
    : initManager(std::make_unique<business::InitManager>(nodeHandle)) {

    initManager->init();
}

}  // namespace twist
