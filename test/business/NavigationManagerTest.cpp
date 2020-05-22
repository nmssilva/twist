/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include <gtest/gtest.h>
#include <geometry_msgs/PoseStamped.h>
#include <common/Logging.h>

#include "../../src/business/flow/NavigationManager.h"
#include "../../src/application/Application.h"

using namespace ::testing;

constexpr auto spinDelayUs{100};

class NavigationManagerTest : public ::testing::Test {
public:
    void SetUp() override {
        publisher = nodeHandle.advertise<std_msgs::String>(twist::business::TargetTopic, 1000);
        victim = std::make_shared<twist::business::NavigationManager>(nodeHandle);
        subscriber = nodeHandle.subscribe(twist::business::MoveGoalTopic, 1000, &NavigationManagerTest::subscriberCallback, this);
    }

    void subscriberCallback(const geometry_msgs::PoseStamped& poseStamped) {
        LOG_INFO("Asserting " << twist::business::locationConversionInverted(currentLocation));
        ASSERT_EQ(poseStamped.pose.position.x, victim->getLocationMap().at(currentLocation).first);
        ASSERT_EQ(poseStamped.pose.position.y, victim->getLocationMap().at(currentLocation).second);
    }

protected:
    twist::business::Location currentLocation;
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

    auto it{victim->getLocationMap().begin()};
    while (it != victim->getLocationMap().end()) {
        std_msgs::String msg;
        currentLocation = it->first;
        msg.data = twist::business::locationConversionInverted(currentLocation);
        publisher.publish(msg);
        usleep(spinDelayUs);
        ros::spinOnce();
        usleep(spinDelayUs);
        ros::spinOnce();

        it++;
    }
}
