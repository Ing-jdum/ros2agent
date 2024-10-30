// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "workshop_plansys2/behavior_tree_nodes/Move.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"


namespace workshop_plansys2
{

Move::Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
: plansys2::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    if (!config().blackboard->get("node", node)) {
        RCLCPP_ERROR(rclcpp::get_logger("Move"), "Failed to get 'node' from the blackboard");
        throw std::runtime_error("Node is required for Move constructor");
    }

    try {
        node->declare_parameter<std::vector<std::string>>("waypoints");
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
        // Do nothing if already declared
    }

    if (node->has_parameter("waypoints")) {
        std::vector<std::string> wp_names;
        node->get_parameter_or("waypoints", wp_names, {});

        for (const auto & wp : wp_names) {
            try {
                node->declare_parameter<std::vector<double>>("waypoint_coords." + wp);
            } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
                // Do nothing if already declared
            }

            std::vector<double> coords;
            if (node->get_parameter_or("waypoint_coords." + wp, coords, {})) {
                geometry_msgs::msg::Pose2D pose;
                pose.x = coords[0];
                pose.y = coords[1];
                pose.theta = coords[2];
                waypoints_[wp] = pose;
            } else {
                RCLCPP_WARN(node->get_logger(), "No coordinate configured for waypoint [%s]", wp.c_str());
            }
        }
    }


    // Initialize the GetEntityState client on the service node
    get_entity_state_client_ = node->create_client<gazebo_msgs::srv::GetEntityState>("/demo/get_entity_state");

    // Use the logger from the service node directly
    while (!get_entity_state_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node->get_logger(), "Waiting for /demo/get_entity_state service to be available...");
    }
}



  BT::NodeStatus
  Move::on_tick()
  { 

    try {
      if (status() == BT::NodeStatus::RUNNING) {
        rclcpp_lifecycle::LifecycleNode::SharedPtr node;
        if (!config().blackboard->get("node", node)) {
          RCLCPP_ERROR(node_->get_logger(), "Failed to get 'node' from the blackboard");
        }

        // Prepare and send the request asynchronously
        auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
        request->name = "burger"; 

        get_entity_state_client_->async_send_request(
        request,
        [node, this](rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedFuture future) {
          try {
            auto result = future.get();
            geometry_msgs::msg::Point current_position = result->state.pose.position;
            RCLCPP_INFO(node->get_logger(), "position: x=%.2f, y=%.2f, z=%.2f",
                        current_position.x, current_position.y, current_position.z);

            // Calculate the distance to the goal
            double distance = calculate_distance(current_position, goal_.pose.pose.position);
            RCLCPP_INFO(node->get_logger(), "Distance to goal: %.2f", distance);

            // Check if distance is less than 0.5 to complete the task
            if (distance < 0.5) {
              // Set the status to SUCCESS to indicate reaching the goal
              this->setStatus(BT::NodeStatus::SUCCESS);
            }
          } catch (const std::exception &e) {
            RCLCPP_ERROR(node->get_logger(), "Failed to get entity state: %s", e.what());
          }
        });
      }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("Move"), "Exception caught in on_tick: %s", e.what());
    }

    // Original logic for other states (IDLE or others)
    if (status() == BT::NodeStatus::IDLE) {
      rclcpp_lifecycle::LifecycleNode::SharedPtr node;
      if (!config().blackboard->get("node", node)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get 'node' from the blackboard");
      }

      std::string goal;
      getInput<std::string>("goal", goal);

      geometry_msgs::msg::Pose2D pose2nav;
      if (waypoints_.find(goal) != waypoints_.end()) {
        pose2nav = waypoints_[goal];
      } else {
        std::cerr << "No coordinate for waypoint [" << goal << "]" << std::endl;
      }

      geometry_msgs::msg::PoseStamped goal_pos;
      goal_pos.header.frame_id = "map";
      goal_pos.header.stamp = node->now();
      goal_pos.pose.position.x = pose2nav.x;
      goal_pos.pose.position.y = pose2nav.y;
      goal_pos.pose.position.z = 0;
      goal_pos.pose.orientation = tf2::toMsg(tf2::Quaternion({0.0, 0.0, 1.0}, pose2nav.theta));

      goal_.pose = goal_pos;
    }

    return BT::NodeStatus::RUNNING;
  }

  double
  Move  ::calculate_distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  BT::NodeStatus
  Move::on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus
  Move::on_aborted()
  {
    RCLCPP_WARN(node_->get_logger(), "Navigation to goal aborted");
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus
  Move::on_cancelled()
  {
    RCLCPP_INFO(node_->get_logger(), "Navigation goal was cancelled");
    return BT::NodeStatus::FAILURE;
  }

}  // namespace workshop_plansys2

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<workshop_plansys2::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<workshop_plansys2::Move>(
    "Move", builder);
}