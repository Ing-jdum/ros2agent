#ifndef WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__UNLOAD_HPP_
#define WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__UNLOAD_HPP_

#include <string>
#include <iostream>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace workshop_plansys2
{

class Unload : public BT::ActionNodeBase
{
public:
  explicit Unload(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
     return {
      BT::InputPort<std::string>("piece"),
      BT::InputPort<std::string>("location")
    };
  }

private:
  int counter_;

  rclcpp::Node::SharedPtr node_;  // Node pointer for ROS 2 communication
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr get_entity_state_client_;  // Client for GetEntityState
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_state_client_;  // Client for SetEntityState

  std::optional<geometry_msgs::msg::Pose> get_position(const std::string &model_name);  // Declare get_position

  BT::NodeStatus teleport_piece(const std::string &piece, const geometry_msgs::msg::Pose &target_pose);  // Declare teleport_piece
  geometry_msgs::msg::Pose get_location_pose(const std::string &location_name);
};

}  // namespace workshop_plansys2

#endif  // WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__UNLOAD_HPP_
