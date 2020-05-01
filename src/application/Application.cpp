/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "Application.h"

#include <ros/node_handle.h>

/**
 * @ingroup twist
 *
 * Namespace for all TWIST modules
 */
namespace twist {

Application::Application(ros::NodeHandle& nodeHandle)
    : initManager(std::make_unique<InitManager>(nodeHandle)) {
}

}  // namespace twist
