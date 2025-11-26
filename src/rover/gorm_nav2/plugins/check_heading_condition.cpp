#include "gorm_nav2/check_heading_condition.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"  // for euclidean or pose helpers if needed
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include <memory>
#include <string>
#include <cmath>

namespace gorm_nav2
{

CheckHeadingCondition::CheckHeadingCondition(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config),
  tolerance_(0.1)
{
  // nothing heavy here; initialization deferred to initialize()
}

void CheckHeadingCondition::initialize()
{
  // get the ROS node and tf_buffer from the BT blackboard (Nav2 sets these)
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  // optional: allow a transform tolerance param if present on the node, otherwise keep default
  double tmp = transform_tolerance_;
  if (node_->get_parameter_or("transform_tolerance", tmp)) {
    transform_tolerance_ = tmp;
  }

  initialized_ = true;
}
bool rotating = false;

BT::NodeStatus CheckHeadingCondition::tick()
{
  if (!initialized_) {
    initialize();
  }

  geometry_msgs::msg::PoseStamped goal;
  if (!getInput("goal", goal)) {
    throw BT::RuntimeError("Missing required input [goal]");
  }
  // get tolerance (allow override from XML port)
  getInput("tolerance", tolerance_);

  // Get current robot pose in the same frame as goal
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
        robot_pose, *tf_, goal.header.frame_id, "base_link", transform_tolerance_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "Current robot pose is not available (CheckHeading).");
    return BT::NodeStatus::FAILURE;
  }

  // Compute yaws
  tf2::Quaternion robot_q(
    robot_pose.pose.orientation.x,
    robot_pose.pose.orientation.y,
    robot_pose.pose.orientation.z,
    robot_pose.pose.orientation.w);
  double robot_yaw = tf2::getYaw(robot_q);

  tf2::Quaternion goal_q(
    goal.pose.orientation.x,
    goal.pose.orientation.y,
    goal.pose.orientation.z,
    goal.pose.orientation.w);
  double goal_yaw = tf2::getYaw(goal_q);

  // smallest angular difference
  double heading_diff = std::fmod(std::fabs(robot_yaw - goal_yaw), 2.0 * M_PI);
  if (heading_diff > M_PI) {
    heading_diff = 2.0 * M_PI - heading_diff;
  }


  BT::NodeStatus current_status;


  if (!rotating){
    if(heading_diff < tolerance_){
      current_status = BT::NodeStatus::SUCCESS;
    } else {
      rotating = true;
      tolerance_ = 0.09;
      current_status = BT::NodeStatus::FAILURE;
    }
  } 
  else{
    if (heading_diff > tolerance_){
      current_status = BT::NodeStatus::FAILURE;
    } else {
      current_status = BT::NodeStatus::SUCCESS;
      tolerance_ = 0.785;
      rotating = false;
    }
  }


   if (current_status != last_status_) {
    if (current_status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "CheckHeadingCondition: heading aligned → USING SMAC HYBRID");
    } else {
      RCLCPP_INFO(node_->get_logger(), "CheckHeadingCondition: heading misaligned → USING NavFN PLANNER");
    }
    last_status_ = current_status;
  }
  return current_status;
}

}  // namespace gorm_nav2

// Export node to BehaviorTree factory using a builder (recommended pattern)
#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<gorm_nav2::CheckHeadingCondition>("CheckHeadingCondition");

}



