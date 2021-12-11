// Copyright 2021 James Yoo
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software without
//    specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef GNSS_WAYPOINT_FOLLOWER__GNSS_WAYPOINT_FOLLOWER_HPP_
#define GNSS_WAYPOINT_FOLLOWER__GNSS_WAYPOINT_FOLLOWER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"


namespace gnss_waypoint_follower
{
/*
 * @class GnssWaypointFollower
 * @brief ROS2 wrapper for gnss waypoint follower
 */
class GnssWaypointFollower : public rclcpp_lifecycle::LifecycleNode
{
public:
  /*
   * @brief GnssWaypointFollower constructor
   * @param options Additional options to control creation of the node.
   */
  explicit GnssWaypointFollower(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /*
   * @brief GnssWaypointFollower destructor
   */
  ~GnssWaypointFollower();
};

}

#endif // GNSS_WAYPOINT_FOLLOWER__GNSS_WAYPOINT_FOLLOWER_HPP_