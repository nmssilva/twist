/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

/**
 * Main used for tests
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "UnitTests");

    return RUN_ALL_TESTS();
}
