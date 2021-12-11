
/* Author: James Yoo */

#include "gnss_waypoint_follower/gnss_waypoint_follower.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace gnss_waypoint_follower
{

GnssWaypointFollower::GnssWaypointFollower(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("gnss_waypoint_follower_lc", options)
{

}

GnssWaypointFollower::~GnssWaypointFollower()
{
}

}