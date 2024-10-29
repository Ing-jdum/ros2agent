#include <string>
#include <iostream>
#include <unordered_map>
#include <optional>

#include "rclcpp/rclcpp.hpp"

#include "workshop_plansys2/behavior_tree_nodes/Unload.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace workshop_plansys2
{

  Unload::Unload(
      const std::string &xml_tag_name,
      const BT::NodeConfiguration &conf)
      : BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
  {
    // Initialize ROS 2 Node and service clients
    node_ = rclcpp::Node::make_shared("unload_node");

    // Initialize GetEntityState client
    get_entity_state_client_ = node_->create_client<gazebo_msgs::srv::GetEntityState>("/demo/get_entity_state");

    // Ensure /demo/get_entity_state service is available
    while (!get_entity_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(node_->get_logger(), "Waiting for /demo/get_entity_state service to be available...");
    }

    // Initialize SetEntityState client
    set_entity_state_client_ = node_->create_client<gazebo_msgs::srv::SetEntityState>("/demo/set_entity_state");

    // Ensure /demo/set_entity_state service is available
    while (!set_entity_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(node_->get_logger(), "Waiting for /demo/set_entity_state service to be available...");
    }
  }

  void
  Unload::halt()
  {
    std::cout << "Unload halt" << std::endl;
  }

  BT::NodeStatus
  Unload::tick()
  {
    // Declare input variables
    std::string piece;
    std::string location;

    // Retrieve dynamic inputs
    if (!getInput<std::string>("piece", piece) || !getInput<std::string>("location", location))
    {
      throw BT::RuntimeError("Missing required input [piece] or [location]");
    }

    // Retrieve position of the target location using the mapping function
    geometry_msgs::msg::Pose location_pose;
    try
    {
      location_pose = get_location_pose(location);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }

    // Retrieve position of the piece
    auto piece_position_opt = get_position(piece);

    if (!piece_position_opt)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get position of the piece.");
      return BT::NodeStatus::FAILURE;
    }

    // Teleport the piece to the target location
    return teleport_piece(piece, location_pose);
  }

  std::optional<geometry_msgs::msg::Pose>
  Unload::get_position(const std::string &model_name)
  {
    auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
    request->name = model_name;

    auto result = get_entity_state_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service /demo/get_entity_state for model %s", model_name.c_str());
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

  // Function to map location names to their coordinates
  geometry_msgs::msg::Pose Unload::get_location_pose(const std::string &location_name)
  {
    // Create a mapping between location names and their coordinates
    static const std::unordered_map<std::string, geometry_msgs::msg::Pose> location_map = {
        {"table_01", []()
         { geometry_msgs::msg::Pose pose; pose.position.x = 1.35; pose.position.y = 1.19; pose.position.z = 0.8; return pose; }()},
        {"table_02", []()
         { geometry_msgs::msg::Pose pose; pose.position.x = 1.9; pose.position.y = -0.19; pose.position.z = 0.8; return pose; }()},
        {"table_03", []()
         { geometry_msgs::msg::Pose pose; pose.position.x = -0.65; pose.position.y = -1.25; pose.position.z = 0.8; return pose; }()},
        {"table_04", []()
         { geometry_msgs::msg::Pose pose; pose.position.x = -1.2; pose.position.y = 0.95; pose.position.z = 0.8; return pose; }()},
        {"table_05", []()
         { geometry_msgs::msg::Pose pose; pose.position.x = -2.0; pose.position.y = -0.28; pose.position.z = 0.8; return pose; }()},
        {"trash", []()
         { geometry_msgs::msg::Pose pose; pose.position.x = -2.75; pose.position.y = 0.65; pose.position.z = 0.015; return pose; }()}};;

    auto it = location_map.find(location_name);
    if (it != location_map.end())
    {
      return it->second;
    }

    throw std::runtime_error("Unknown location name: " + location_name);
  }

  // Function to teleport a piece to a specific location
  BT::NodeStatus Unload::teleport_piece(const std::string &piece, const geometry_msgs::msg::Pose &target_pose)
  {
    auto client = node_->create_client<gazebo_msgs::srv::SetEntityState>("/demo/set_entity_state");

    // Wait for the service to be available
    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Service /demo/set_entity_state is not available.");
      return BT::NodeStatus::FAILURE;
    }

    // Create the request
    auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
    request->state.name = piece;
    request->state.pose = target_pose;

    // Call the service
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "Successfully teleported piece [%s] to the target location.", piece.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to teleport piece [%s] to the target location.", piece.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

} // namespace workshop_plansys2

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<workshop_plansys2::Unload>("Unload");
}
