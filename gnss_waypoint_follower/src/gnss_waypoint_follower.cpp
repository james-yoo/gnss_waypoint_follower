
/* Author: James Yoo */

#include "gnss_waypoint_follower/gnss_waypoint_follower.hpp"

namespace gnss_waypoint_follower
{

GnssWaypointFollower::GnssWaypointFollower(
  const std::string & node_name,
  bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(
  node_name,
  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{

}

GnssWaypointFollower::~GnssWaypointFollower()
{
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_configure() is called.");

  // TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
  // TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_activate() is called.");

  // TRANSITION_CALLBACK_SUCCESS transitions to "active"
  // TRANSITION_CALLBACK_FAILURE transitions to "inactive"
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

  // TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
  // TRANSITION_CALLBACK_FAILURE transitions to "active"
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_cleanup() is called.");

  // TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
  // TRANSITION_CALLBACK_FAILURE transitions to "inactive"
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_shutdown() is called.");

  // TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
  // TRANSITION_CALLBACK_FAILURE transitions to current state
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // amespace gnss_waypoint_follower