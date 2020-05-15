/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "../../src/common/include/common/Mapping.h"

#include <gtest/gtest.h>

using namespace ::testing;

const auto intStringConversion{twist::common::enumMapper<int, std::string>({{1, "1"}, {3, "3"}, {9, "9"}})};

/**
 * objectNotNull: assert object creation succeeded
 */
TEST(MappingTest, objectNotNull) {
    ASSERT_EQ(intStringConversion(1), "1");
    ASSERT_EQ(intStringConversion(3), "3");
    ASSERT_EQ(intStringConversion(9), "9");
}
