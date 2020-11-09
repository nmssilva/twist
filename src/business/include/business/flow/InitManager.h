/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#pragma once

#include <ros/node_handle.h>

#include "../../../flow/NavigationManager.h"

/**
 * @ingroup twist
 *
 * Namespace for all TWIST modules
 */
namespace twist {

/**
 * @ingroup twist
 *
 * Namespace for all business modules
 */
namespace business {

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

    /**
     * Initializer for each of the managers
     */
    void init();

    /**
     * Initializer fail handler
     */
    void initFailure();

private:
    ros::NodeHandle nodeHandle;

    std::shared_ptr<NavigationManager> navigationManager;
};

}  // namespace business
}  // namespace twist
