#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"

#include "workshop_plansys2/behavior_tree_nodes/Load.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace workshop_plansys2
{

  Load::Load(
      const std::string &xml_tag_name,
      const BT::NodeConfiguration &conf)
      : BT::ActionNodeBase(xml_tag_name, conf)
  {
    // Initialize ROS 2 Node and service clients
    node_ = rclcpp::Node::make_shared("load_node");

    // Initialize GetEntityState client
    get_entity_state_client_ = node_->create_client<gazebo_msgs::srv::GetEntityState>("/demo/get_entity_state");

    // Ensure /gazebo/get_entity_state service is available
    while (!get_entity_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(node_->get_logger(), "Waiting for /demo/get_entity_state service to be available...");
    }

    // Initialize SetEntityState client
    set_entity_state_client_ = node_->create_client<gazebo_msgs::srv::SetEntityState>("/demo/set_entity_state");

    // Ensure /gazebo/set_entity_state service is available
    while (!set_entity_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(node_->get_logger(), "Waiting for /demo/set_entity_state service to be available...");
    }
  }

  BT::NodeStatus
  Load::tick()
  {
    // Declare input variables
    std::string piece;
    std::string robot;

    // Retrieve dynamic inputs
    if (!getInput<std::string>("piece", piece) || !getInput<std::string>("robot", robot))
    {
      throw BT::RuntimeError("Missing required input [piece] or [robot]");
    }

    auto robot_position_opt = get_position(robot);
    auto piece_position_opt = get_position(piece);

    if (!robot_position_opt)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get position of the robot.");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Pose robot_pose = *robot_position_opt;

    if (!piece_position_opt)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get position of the piece.");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Pose piece_pose = *piece_position_opt;

    double distance = calculate_distance(robot_pose.position, piece_pose.position);

    if (distance > 2.0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Object not at specified location");
      return BT::NodeStatus::FAILURE;
    }

    std::cout << "Picking up piece: " << piece << " with robot: " << robot << std::endl;

    // Implement teleport logic using SetEntityState service
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

    request->state.name = piece;
    request->state.pose.position.x = 5.0; // Hardcode x position
    request->state.pose.position.y = 5.0; // Hardcode y position
    request->state.pose.position.z = 0.0; // Hardcode z postiion

    auto result_future = set_entity_state_client_->async_send_request(request);

    // Wait for the result or handle as necessary
    if (rclcpp::spin_until_future_complete(node_, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_entity_state");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

  std::optional<geometry_msgs::msg::Pose>
  Load::get_position(const std::string &model_name)
  {
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = model_name;

    auto result = get_entity_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service /gazebo/get_entity_state for model %s", model_name.c_str());
      return std::nullopt;
    }

    auto response = result.get();
    if (response->success)
    {
      return response->state.pose;
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Could not get state for model %s", model_name.c_str());
      return std::nullopt;
    }
  }

  double
  Load::calculate_distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void
  Load::halt()
  {
    std::cout << "Load halt" << std::endl;
  }

} // namespace workshop_plansys2

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<workshop_plansys2::Load>("Load");
}