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

/**
 * Different locations around the house
 */
enum class Location { Kitchen1, Kitchen2, DinningRoom, LivingRoom, BedRoom1, BedRoom2, Outside, Hall };

/**
 * Mapper for the Location into std::string
 */
const auto locationConversion{common::enumMapper<std::string, Location>({{"Kitchen1", Location::Kitchen1},
                                                                                 {"Kitchen2", Location::Kitchen2},
                                                                                 {"DinningRoom", Location::DinningRoom},
                                                                                 {"LivingRoom", Location::LivingRoom},
                                                                                 {"BedRoom1", Location::BedRoom1},
                                                                                 {"BedRoom2", Location::BedRoom2},
                                                                                 {"Outside", Location::Outside},
                                                                                 {"Hall", Location::Hall}})};

/**
 * Mapper for the std::string into Location
 */
const auto locationConversionInverted {common::enumMapper<Location, std::string>({{Location::Kitchen1, "Kitchen1"},
                                                                                 {Location::Kitchen2, "Kitchen2"},
                                                                                 {Location::DinningRoom, "DinningRoom"},
                                                                                 {Location::LivingRoom, "LivingRoom"},
                                                                                 {Location::BedRoom1, "BedRoom1"},
                                                                                 {Location::BedRoom2, "BedRoom2"},
                                                                                 {Location::Outside, "Outside"},
                                                                                 {Location::Hall, "Hall"}})};
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

    /**
     * Getter for location map, for test purposes
     * @return location map
     */
    [[nodiscard]] std::map<Location, std::pair<x, y>> &getLocationMap();

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

    /**
     * Map between locations and coordinates
     */
    std::map<Location, std::pair<x, y>> LocationMap{{Location::Kitchen1, std::make_pair(3.3, 2.2)},
                                                    {Location::Kitchen2, std::make_pair(1.1, 3.2)},
                                                    {Location::LivingRoom, std::make_pair(-1.3, 2.3)},
                                                    {Location::DinningRoom, std::make_pair(5.6, -4.4)},
                                                    {Location::BedRoom1, std::make_pair(-6.1, 3.2)},
                                                    {Location::BedRoom2, std::make_pair(-6.4, -0.7)},
                                                    {Location::Outside, std::make_pair(-0.7, -3.5)},
                                                    {Location::Hall, std::make_pair(0.3, 0.3)}};
};

}  // namespace business
}  // namespace twist
