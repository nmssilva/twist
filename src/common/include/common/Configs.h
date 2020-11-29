/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#pragma once

#include <tuple>
#include <vector>

#include <common/Mapping.h>
#include <ros/ros.h>

namespace twist {

/**
 * @ingroup common
 * Namespace for common utilities
 */
namespace common {

/**
 * Anonymous enumeration to avoid magic numbers
 */
enum { x, y, z, w};

/**
 * Different locations around the house
 */
enum class Location { BedRoomKids, BedRoomSuite, DinningRoom, Hall, Kitchen, LivingRoom, WC, WCSuite };

/**
 * Mapper for the std::string into Location
 */
const auto locationConversionInverted{common::enumMapper<Location, std::string>({
    {Location::BedRoomKids, "BedRoomKids"},
    {Location::BedRoomSuite, "BedRoomSuite"},
    {Location::DinningRoom, "DinningRoom"},
    {Location::Hall, "Hall"},
    {Location::Kitchen, "Kitchen"},
    {Location::LivingRoom, "LivingRoom"},
    {Location::WC, "WC"},
    {Location::WCSuite, "WCSuite"},
})};

/**
 * Mapper for the Location into std::string
 */
const auto locationConversion{common::enumMapper<std::string, Location>({{"BedRoomKids", Location::BedRoomKids},
                                                                         {"BedRoomSuite", Location::BedRoomSuite},
                                                                         {"DinningRoom", Location::DinningRoom},
                                                                         {"Hall", Location::Hall},
                                                                         {"Kitchen", Location::Kitchen},
                                                                         {"LivingRoom", Location::LivingRoom},
                                                                         {"WC", Location::WC},
                                                                         {"WCSuite", Location::WCSuite}})};

/**
 * Struct that mimics the toml configuration.
 */
struct ConfigValues {
    /**
    * Location coordinates
    */
    struct Coordinates {
        double x, y, z, w;
    };

    /**
     * Struct that holds the locations.
     */
    std::vector<Coordinates> locations;
};

class [[nodiscard]] Configs {
public:
    /**
     * Parse the config file
     *
     * @param configFile path for the configuration file
     * @return status of parsing
     */
    //[[nodiscard]] static bool parseConfigFile(const std::string& configFile = std::string(CONFIG_FILE));
    [[nodiscard]] static bool parseConfigFile(const std::string& configFile = std::string(CONFIG_FILE));

    /**
     * Getter for the toml configurations
     * @return collection of values parsed from toml file
     */
    [[nodiscard]] static const ConfigValues& getValues();

private:
    /**
     * Collection of configuration values parsed from toml, kept in RAM.
     */
    static ConfigValues values;
};

}  // namespace common
}  // namespace twist
