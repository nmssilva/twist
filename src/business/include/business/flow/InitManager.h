/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#pragma once

#include <ros/node_handle.h>

/**
 * @ingroup twist
 *
 * Namespace for all TWIST modules
 */
namespace twist {

class InitManager {

public:
    /**
     * Constructor
     *
     * @param nodeHandle node handle from main
     */
    explicit InitManager(ros::NodeHandle& nodeHandle);

    /**
     * Destructor default
     */
    ~InitManager() = default;

private:
    ros::NodeHandle nodeHandle;

};

}  // namespace application
