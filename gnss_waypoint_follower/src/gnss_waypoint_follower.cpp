
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
  RCLCPP_INFO(get_logger(), "Creating lifecycle node");

  declare_parameter("map_frame", "map");
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_link_frame", "base_link");
  declare_parameter("waypoints_file", "default.json");
  declare_parameter("topic_cmd_vel", "cmd_vel");
  declare_parameter("include_start_pose", true);
  declare_parameter("linear_velocity", 0.5);
  declare_parameter("angular_velocity", 0.5);
  declare_parameter("goal_distance", 1.0);
}

GnssWaypointFollower::~GnssWaypointFollower()
{
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_configure()");

  // Get parameters
  map_frame_ = get_parameter("map_frame").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_link_frame_ = get_parameter("base_link_frame").as_string();
  waypoints_file_ = get_parameter("waypoints_file").as_string();
  topic_cmd_vel_ = get_parameter("topic_cmd_vel").as_string();
  include_start_pose_ = get_parameter("include_start_pose").as_bool();
  linear_velocity_ = get_parameter("linear_velocity").as_double();
  angular_velocity_ = get_parameter("angular_velocity").as_double();
  goal_distance_ = get_parameter("goal_distance").as_double();

  // Create publishers
  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, 10);

  // Create subscribers

  // TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
  // TRANSITION_CALLBACK_FAILURE transitions to "unconfigured"
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_activate()");

  pub_cmd_vel_->on_activate();
  // TRANSITION_CALLBACK_SUCCESS transitions to "active"
  // TRANSITION_CALLBACK_FAILURE transitions to "inactive"
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_deactivate()");

  pub_cmd_vel_->on_deactivate();

  // TRANSITION_CALLBACK_SUCCESS transitions to "inactive"
  // TRANSITION_CALLBACK_FAILURE transitions to "active"
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_cleanup()");

  pub_cmd_vel_.reset();

  // TRANSITION_CALLBACK_SUCCESS transitions to "unconfigured"
  // TRANSITION_CALLBACK_FAILURE transitions to "inactive"
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn
GnssWaypointFollower::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "on_shutdown()");

  pub_cmd_vel_.reset();

  // TRANSITION_CALLBACK_SUCCESS transitions to "finalized"
  // TRANSITION_CALLBACK_FAILURE transitions to current state
  // TRANSITION_CALLBACK_ERROR or any uncaught exceptions to "errorprocessing"
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

} // amespace gnss_waypoint_follower