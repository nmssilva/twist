/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include <cstdlib>

#include "Application.h"

#include "ros/ros.h"

/**
 * Application entry point
 * @param argc argc
 * @param argv argv
 * @return 0
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "application");

    ros::NodeHandle nodeHandle;

    ROS_INFO("TWIST bring-up...");

    // Application is created, and is only destroyed when we stop spinning
    twist::Application application(nodeHandle);

    ros::spin();

    return EXIT_SUCCESS;
}
