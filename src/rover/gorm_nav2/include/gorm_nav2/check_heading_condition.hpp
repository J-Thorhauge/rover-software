// check_heading_condition.hpp  (outline)
#ifndef GORM_NAV2__CHECK_HEADING_CONDITION_HPP_
#define GORM_NAV2__CHECK_HEADING_CONDITION_HPP_

#include <memory>
#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace gorm_nav2
{

class CheckHeadingCondition : public BT::ConditionNode
{
public:
  CheckHeadingCondition(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts()
  {
    return BT::PortsList{
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<double>("tolerance")
    };
  }

  BT::NodeStatus tick() override;

private:
  void initialize();

  bool initialized_{false};
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double tolerance_;
  double transform_tolerance_{0.1};  // keep a small default
  BT::NodeStatus last_status_ = BT::NodeStatus::IDLE;

};

}  // namespace gorm_nav2

#endif  // GORM_NAV2__CHECK_HEADING_CONDITION_HPP_
