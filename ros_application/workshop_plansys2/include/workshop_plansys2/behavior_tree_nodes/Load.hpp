#ifndef WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__LOAD_HPP_
#define WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__LOAD_HPP_

#include <string>
#include <optional>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"  // Include GetEntityState service
#include "geometry_msgs/msg/pose.hpp"

namespace workshop_plansys2
{

class Load : public BT::ActionNodeBase
{
public:
  explicit Load(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() override;
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("piece"),
      BT::InputPort<std::string>("robot")
    };
  }

private:
  int counter_;
  rclcpp::Node::SharedPtr node_;  // Node pointer for ROS 2 communication
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr get_entity_state_client_;  // Client for GetEntityState
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_client_;  // Client for GetEntityState

  std::optional<geometry_msgs::msg::Pose> get_position(const std::string &model_name);  // Declare get_position
  double calculate_distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2);
};

}  // namespace workshop_plansys2

#endif  // WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__LOAD_HPP_
