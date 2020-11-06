/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#pragma once

#include <ros/node_handle.h>

#include "common/Mapping.h"
#include "std_msgs/String.h"

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

/**
 * Topic for movement goal publishing
 */
constexpr auto MoveGoalTopic{"/move_base_simple/goal"};

/**
 * Topic for movement goal publishing
 */
constexpr auto TargetTopic{"/target"};

/**
 * x coordinates type
 */
using x = double;

/**
 * y coordinates type
 */
using y = double;

class NavigationManager {
public:
    /**
     * Constructor
     *
     * @param nodeHandle node handle from main
     */
    explicit NavigationManager(ros::NodeHandle& nodeHandle);

    /**
     * Destructor default
     */
    ~NavigationManager() = default;

private:
    /**
     * Callback to set the goal based on target provided
     *
     * * @param target
     */
    void setGoalFromTargetCallback(const std_msgs::String::ConstPtr& target);

private:
    ros::NodeHandle nodeHandle;
    ros::Publisher moveGoalPublisher;
    ros::Subscriber getTargetSubscriber;

};

}  // namespace business
}  // namespace twist
