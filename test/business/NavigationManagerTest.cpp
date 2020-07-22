/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "../../src/business/flow/NavigationManager.h"

#include <geometry_msgs/PoseStamped.h>
#include <gtest/gtest.h>

#include "../../src/application/Application.h"
#include "common/Configs.h"
#include "common/Logging.h"

using namespace ::testing;

constexpr auto spinDelayUs{100};

class NavigationManagerTest : public ::testing::Test {
public:
    void SetUp() override {
        if (twist::common::Configs::parseConfigFile()) {
            publisher = nodeHandle.advertise<std_msgs::String>(twist::business::TargetTopic, 1000);
            victim = std::make_shared<twist::business::NavigationManager>(nodeHandle);
            subscriber = nodeHandle.subscribe(twist::business::MoveGoalTopic, 1000, &NavigationManagerTest::subscriberCallback, this);
        } else {
            LOG_ERROR("Failed to parse configs");
            ASSERT_TRUE(false);
        }
    }

    void subscriberCallback(const geometry_msgs::PoseStamped& poseStamped) {
        LOG_INFO("Asserting " << twist::common::locationConversionInverted(currentLocation));

        const auto locations{twist::common::Configs::getValues().locations.at(static_cast<uint32_t>(currentLocation))};
        ASSERT_EQ(poseStamped.pose.position.x, locations.x);
        ASSERT_EQ(poseStamped.pose.position.y, locations.y);
        ASSERT_EQ(poseStamped.pose.orientation.z, locations.z);
        ASSERT_EQ(poseStamped.pose.orientation.w, locations.w);
    }

protected:
    twist::common::Location currentLocation;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    ros::NodeHandle nodeHandle;
    std::shared_ptr<twist::business::NavigationManager> victim;
};

/**
 * objectNotNull: assert object creation succeeded
 */
TEST_F(NavigationManagerTest, objectNotNull) {
    ASSERT_NE(victim, nullptr);
}

/**
 * topicConversionTest: verify that the target-location conversion works as expected
 */
TEST_F(NavigationManagerTest, topicConversionTest) {
    ASSERT_EQ(1, subscriber.getNumPublishers());
    ASSERT_EQ(1, publisher.getNumSubscribers());

    const auto locationList{twist::common::Configs::getValues().locations};

    for(auto location_idx{0}; location_idx < 8; location_idx++) {
        std_msgs::String msg;
        currentLocation = static_cast<twist::common::Location>(location_idx);
        msg.data = twist::common::locationConversionInverted(currentLocation);
        publisher.publish(msg);
        usleep(spinDelayUs);
        ros::spinOnce();
        usleep(spinDelayUs);
        ros::spinOnce();
    }

    // Test invalid location
    {
        std_msgs::String msg;
        msg.data = "China";
        publisher.publish(msg);
        usleep(spinDelayUs);
        ros::spinOnce();
    }
}
