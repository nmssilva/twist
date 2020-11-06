/**
 * \file
 * \brief       Application node
 *
 * \project     TWIST
 * \copyright   FIKALAB
 */

#include "common/Configs.h"

#include <cpptoml/cpptoml.h>

#include "common/Logging.h"

namespace twist {
namespace common {

/**
 * Definition of static configuration holder
 */
ConfigValues Configs::values;

/**
 * Extracts location values
 *
 * @param locations_table locations table
 * @param values holds for parsed values
 * */
static void extractLocations(const std::shared_ptr<cpptoml::table>& locations_table, ConfigValues& values);

bool Configs::parseConfigFile(const std::string& configFile) {
    LOG_INFO("Parsing config file");

    try {
        const auto config{cpptoml::parse_file(configFile)};
        extractLocations(config->get_table("locations"), values);
    } catch (const cpptoml::parse_exception& exception) {
        LOG_ERROR("Exception parsing config file:" << exception.what());
        return false;
    }

    return true;
}

void extractLocations(const std::shared_ptr<cpptoml::table>& locations_table, ConfigValues& values) {
    const auto extractLocations{[&](const Location& location) {
        const auto location_coordinates{*locations_table->get_array_of<double>(locationConversionInverted(location))};
        values.locations.emplace_back(ConfigValues::Coordinates{location_coordinates.at(x),
                                                                location_coordinates.at(y),
                                                                location_coordinates.at(z),
                                                                location_coordinates.at(w)});
    }};

    if (!!locations_table) {
        extractLocations(Location::BedRoomKids);
        extractLocations(Location::BedRoomSuite);
        extractLocations(Location::DinningRoom);
        extractLocations(Location::Hall);
        extractLocations(Location::Kitchen);
        extractLocations(Location::LivingRoom);
        extractLocations(Location::WC);
        extractLocations(Location::WCSuite);
    }
}

const ConfigValues& Configs::getValues() {
    return values;
}

}  // namespace common
}  // namespace twist
