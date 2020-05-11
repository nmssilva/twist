/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include <gtest/gtest.h>

#include "business/flow/InitManager.h"

using namespace ::testing;

class InitManagerTest : public ::testing::Test {
public:
    void SetUp() override {
        victim = std::make_shared<twist::business::InitManager>(nodeHandle);
    }

protected:
    ros::NodeHandle nodeHandle{};
    std::shared_ptr<twist::business::InitManager> victim;
};

/**
 * DummyTest: dummyTest.
 */
TEST_F(InitManagerTest, DummyTest) {
    ASSERT_NE(victim, nullptr);
}
