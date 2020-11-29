/**
 * \file
 * \brief       SFA Master
 *
 * \project     BMW Platform Software
 * \copyright   Critical TechWorks SA
 */

#include "common/Configs.h"

#include <string>

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <fstream>
#include <common/Logging.h>

using namespace twist::common;

static constexpr auto ConfigContentOK{R"(
# This TOML document contains the office team locations.
title = "Office locations"
# x, y, z, w
[locations]
  BedRoomKids = [ -6.25, 3.16, -0.68, 0.72]
  BedRoomSuite = [ -6.25, -0.72, -0.70, 0.72]
  DinningRoom = [ -2.50, 2.44, 0.73, 0.68]
  Hall = [ 1.03, 0.46, -0.7, 0.71]
  Kitchen = [ 4.87, 2.42, 0.70, 0.71]
  LivingRoom = [ -0.84, 3.10, 0.75, 0.66]
  WC = [ 0.98, 2.87, -0.73, 0.67]
  WCSuite = [ 5.92, -2.21, -0.72, 0.69]
)"};

static constexpr auto ConfigContentInvalidData{R"(
# This TOML document contains the office team locations.
title = "Office locations"
# x, y, z, w
[locations]
  BedRoomKids = [ "INVALID", 3.16, -0.68, 0.72]
  BedRoomSuite = [ -6.25, -0.72, -0.70, 0.72]
  DinningRoom = [ -2.50, 2.44, 0.73, 0.68]
  Hall = [ 1.03, 0.46, -0.7, 0.71]
  Kitchen = [ 4.87, 2.42, 0.70, 0.71]
  LivingRoom = [ -0.84, 3.10, 0.75, 0.66]
  WC = [ 0.98, 2.87, -0.73, 0.67]
  WCSuite = [ 5.92, -2.21, -0.72, 0.69]
)"};

static constexpr auto ConfigContentInvalidSection{R"(
# This TOML document contains the office team locations.
title = "Office locations"
# x, y, z, w
[lolcations]
  BedRoomKids = [ "INVALID", 3.16, -0.68, 0.72]
  BedRoomSuite = [ -6.25, -0.72, -0.70, 0.72]
  DinningRoom = [ -2.50, 2.44, 0.73, 0.68]
  Hall = [ 1.03, 0.46, -0.7, 0.71]
  Kitchen = [ 4.87, 2.42, 0.70, 0.71]
  LivingRoom = [ -0.84, 3.10, 0.75, 0.66]
  WC = [ 0.98, 2.87, -0.73, 0.67]
  WCSuite = [ 5.92, -2.21, -0.72, 0.69]
)"};

class ConfigsTest : public ::testing::Test {
public:
    ConfigsTest() {}

    void SetUp() override {
        locationsFile = std::tmpfile();
        locationsFilePath = std::to_string(fileno(locationsFile));
    }

    bool createFile(const std::string& absoluteFilePath, const std::string& content) {
        std::ofstream file(absoluteFilePath);

        if (!file.is_open()) {
            LOG_ERROR("Couldn't create file: " << absoluteFilePath);
            return false;
        } else {
            LOG_INFO("Created file:" << absoluteFilePath);
        }

        file << content;
        file.close();

        return true;
    }

protected:
    std::FILE* locationsFile;
    std::string locationsFilePath;
};

/**
 * validateConfigsParser: feed a locations.toml to the Configs class and check the parsed values.
 */
TEST_F(ConfigsTest, validateConfigsParser) {
    ASSERT_TRUE(createFile(locationsFilePath, ConfigContentOK));
    ASSERT_TRUE(Configs::parseConfigFile(locationsFilePath));

    ASSERT_EQ(Configs::getValues().locations.at(0).x, -6.25);
    ASSERT_EQ(Configs::getValues().locations.at(0).y, 3.16);
    ASSERT_EQ(Configs::getValues().locations.at(0).z, -0.68);
    ASSERT_EQ(Configs::getValues().locations.at(0).w, 0.72);

    ASSERT_EQ(Configs::getValues().locations.at(7).x, 5.92);
    ASSERT_EQ(Configs::getValues().locations.at(7).y, -2.21);
    ASSERT_EQ(Configs::getValues().locations.at(7).z, -0.72);
    ASSERT_EQ(Configs::getValues().locations.at(7).w, 0.69);
}

/**
 * parseNonExistingConfigs: feed the parser with non-existing files, and assert it fails
 */
TEST_F(ConfigsTest, parseUnexistingConfigs) {
    ASSERT_FALSE(Configs::parseConfigFile("locationsFile"));
}

/**
 * parseInvalidConfigData: feed a locations.toml with invalid data to the parser and assert
 * it fails
 */
TEST_F(ConfigsTest, parseInvalidConfigData) {
    ASSERT_TRUE(createFile(locationsFilePath, ConfigContentInvalidData));
    ASSERT_FALSE(Configs::parseConfigFile(locationsFilePath));
}

/**
 * parseInvalidConfigSection: feed a locations.toml with wrong section values to the parser and assert
 * it fails
 */
TEST_F(ConfigsTest, parseInvalidConfigSection) {
    ASSERT_TRUE(createFile(locationsFilePath, ConfigContentInvalidSection));
    ASSERT_FALSE(Configs::parseConfigFile(locationsFilePath));
}
