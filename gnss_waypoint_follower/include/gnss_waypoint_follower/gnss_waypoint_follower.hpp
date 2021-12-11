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
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::placeholders;
using namespace std::chrono_literals;
using namespace rclcpp_lifecycle::node_interfaces;

namespace gnss_waypoint_follower
{
/*
 * @class GnssWaypointFollower
 * @brief ROS2 wrapper for gnss waypoint follower
 */
class GnssWaypointFollower : public rclcpp_lifecycle::LifecycleNode
{
public:

  // @brief GnssWaypointFollower constructor
  // @param node name
  // @param use intra process communication if true
  explicit GnssWaypointFollower(
    const std::string & node_name,
    bool intra_process_comms = false
  );

  // @brief GnssWaypointFollower destructor
  ~GnssWaypointFollower();

protected:

  // @brief being called when the lifecycle node enters the "configuring" state.
  LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  // @brief being called when the lifecycle node enters the "activating" state.
  LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  // @brief being called when the lifecycle node enters the "deactivating" state
  LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

  // @brief being called when the lifecycle node enters the "cleaningup" state
  LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state);

  // @brief being called when the lifecycle node enters the "shuttingdown" state
  LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state);
};

}

#endif // GNSS_WAYPOINT_FOLLOWER__GNSS_WAYPOINT_FOLLOWER_HPP_