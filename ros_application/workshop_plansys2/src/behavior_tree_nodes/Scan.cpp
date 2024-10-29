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
#include <map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "gazebo_msgs/srv/get_model_list.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_pddl_parser/Utils.h"

#include "geometry_msgs/msg/point.hpp"

#include "workshop_plansys2/behavior_tree_nodes/Scan.hpp"

#include "behaviortree_cpp/behavior_tree.h"

namespace workshop_plansys2
{

  // Define the zones with their coordinates
  const std::map<std::string, std::vector<double>> ZONES = {
      {"entrance", {2.4, -2.15, 0.0}},
      {"table_01", {1.64, 0.76, 0.0}},
      {"table_02", {1.7, 0.37, 0.0}},
      {"table_03", {-0.57, -0.82, 0.0}},
      {"table_04", {-1.16, 1.42, 0.0}},
      {"table_05", {-2.0, 0.2, 0.0}},
      {"recharge_zone", {1.86, -1.9, 0.0}},
      {"init_zone", {0.0, 0.0, 0.0}},
      {"trash", {-2.2, 0.76, 0.0}}};

  Scan::Scan(
      const std::string &xml_tag_name,
      const BT::NodeConfiguration &conf)
      : BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
  {
    // Initialize ROS 2 Node
    node_ = rclcpp::Node::make_shared("scan_node");

    // Create service client for /get_model_list
    get_model_list_client_ = node_->create_client<gazebo_msgs::srv::GetModelList>("/get_model_list");

    // Ensure service is available
    while (!get_model_list_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(node_->get_logger(), "Waiting for /get_model_list service to be available...");
    }

    // Create service client for /demo/get_entity_state
    get_entity_state_client_ = node_->create_client<gazebo_msgs::srv::GetEntityState>("/demo/get_entity_state");

    // Ensure /demo/get_entity_state service is available
    while (!get_entity_state_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(node_->get_logger(), "Waiting for /demo/get_entity_state service to be available...");
    }

    // Properly initializing problem_client_ here
    problem_client_ = std::make_shared<plansys2::ProblemExpertClient>();
  }

  void
  Scan::halt()
  {
    std::cout << "Scan halt" << std::endl;
  }

  BT::NodeStatus
  Scan::tick()
  {
    std::cout << "Scan tick " << counter_ << std::endl;

    // Call /get_model_list service
    auto request = std::make_shared<gazebo_msgs::srv::GetModelList::Request>();

    // Get the robot position
    auto robot_position_opt = get_position("burger"); // Change "burger" to your model name
    if (!robot_position_opt)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get position of the robot.");
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::Pose robot_pose = *robot_position_opt;

    // Make asynchronous service call to get models
    auto result = get_model_list_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service /get_model_list");
      return BT::NodeStatus::FAILURE;
    }

    // Extract and print model names from response
    auto response = result.get();
    if (response->model_names.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No models found in simulation.");
      return BT::NodeStatus::FAILURE;
    }

    delete_zone_predicates(robot_pose);

    RCLCPP_INFO(node_->get_logger(), "Models in simulation:");
    for (const auto &model_name : response->model_names)
    {
      if (model_name == "burger")
        continue;

      auto model_position_opt = get_position(model_name);
      if (!model_position_opt)
      {
        RCLCPP_WARN(node_->get_logger(), "Could not get state for model %s", model_name.c_str());
        continue;
      }

      geometry_msgs::msg::Pose model_pose = *model_position_opt;
      double distance = calculate_distance(robot_pose.position, model_pose.position);

      if (distance <= 2.0)
      { // Radius of 2 meters
        RCLCPP_INFO(node_->get_logger(), "Model: %s is within 2 meters radius at Position: (%.2f, %.2f, %.2f)",
                    model_name.c_str(),
                    model_pose.position.x,
                    model_pose.position.y,
                    model_pose.position.z);

        // Add the model to the knowledge base
        add_to_knowledge_base(model_name, model_pose);
      }
    }

    add_robot_to_nearest_zone(robot_pose);

    return BT::NodeStatus::SUCCESS;
  }

  std::optional<geometry_msgs::msg::Pose>
  Scan::get_position(const std::string &model_name)
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

  double
  Scan::calculate_distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void
  Scan::delete_zone_predicates(const geometry_msgs::msg::Pose &robot_pose)
  {
    if (!problem_client_)
    { // Ensure problem_client_ is valid
      RCLCPP_ERROR(node_->get_logger(), "Problem client is not initialized or is invalid.");
      return;
    }

    // Find the closest zone to the robot's current position
    std::string closest_zone = find_closest_zone(robot_pose.position);

    RCLCPP_INFO(
        node_->get_logger(),
        "Closest zone to robot is: %s",
        closest_zone.c_str());

    // Retrieve existing predicates from the knowledge base
    auto predicates = problem_client_->getPredicates();

    // Check if predicates retrieval was successful
    if (predicates.empty())
    {
      RCLCPP_ERROR(node_->get_logger(), "No predicates retrieved from the knowledge base.");
      return;
    }

    // Log all predicates for debugging
    std::ostringstream os;
    os << "Predicates: " << predicates.size() << std::endl;
    for (const auto & predicate : predicates) {
      os << parser::pddl::toString(predicate) << std::endl;
    }
    // Print the contents of os using the node logger
    RCLCPP_INFO(node_->get_logger(), "%s", os.str().c_str());


    // Loop through all predicates and remove those related to the closest zone
    for (const auto &predicate : predicates)
    {
      // Check if the predicate is related to the zone
      if (predicate.name == "piece_at" && predicate.parameters.size() == 2)
      {
        RCLCPP_INFO(
            node_->get_logger(),
            "Predicates: %s %s %s",
            predicate.name.c_str(),
            predicate.parameters[1].name.c_str(),
            predicate.parameters[0].name.c_str());
        if (predicate.parameters[1].name == closest_zone)
        {
          // Remove the existing predicate
          try
          {
            bool success = problem_client_->removePredicate(predicate);

            if (success)
            {
              RCLCPP_INFO(
                  node_->get_logger(),
                  "Removed existing predicate: %s %s %s",
                  predicate.name.c_str(),
                  predicate.parameters[0].name.c_str(),
                  predicate.parameters[1].name.c_str());
            }
            else
            {
              RCLCPP_WARN(
                  node_->get_logger(),
                  "Failed to remove existing predicate: %s %s %s",
                  predicate.name.c_str(),
                  predicate.parameters[0].name.c_str(),
                  predicate.parameters[1].name.c_str());
            }
          }
          catch (const rclcpp::exceptions::RCLError &e)
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to remove predicate: %s", e.what());
          }
        }
      }
    }
  }


void
Scan::add_robot_to_nearest_zone(const geometry_msgs::msg::Pose &pose)
{
  // Find the closest zone
  std::string closest_zone = find_closest_zone(pose.position);

  // Check for existing 'robot_at' predicate and delete it if present
  auto predicates = problem_client_->getPredicates();

  for (const auto &predicate : predicates)
  {
    // Check if the predicate is 'robot_at' for the robot "burger"
    if (predicate.name == "robot_at" && predicate.parameters.size() == 2)
    {
      if (predicate.parameters[0].name == "burger")
      {
        // Remove the existing 'robot_at' predicate
        if (problem_client_->removePredicate(predicate))
        {
          RCLCPP_INFO(
              node_->get_logger(),
              "Removed existing predicate: [robot_at %s %s]",
              predicate.parameters[0].name.c_str(),
              predicate.parameters[1].name.c_str());
        }
        else
        {
          RCLCPP_WARN(
              node_->get_logger(),
              "Failed to remove existing predicate: [robot_at %s %s]",
              predicate.parameters[0].name.c_str(),
              predicate.parameters[1].name.c_str());
        }
      }
    }
  }

  // Add the new 'robot_at' predicate to represent the robot's position in the nearest zone
  plansys2::Predicate new_predicate;
  new_predicate.node_type = plansys2_msgs::msg::Node::PREDICATE;
  new_predicate.name = "robot_at";
  new_predicate.parameters.push_back(parser::pddl::fromStringParam("burger"));
  new_predicate.parameters.push_back(parser::pddl::fromStringParam(closest_zone));

  if (!problem_client_->addPredicate(new_predicate))
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not add predicate [robot_at burger %s]", closest_zone.c_str());
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Added predicate [robot_at burger %s] to knowledge base", closest_zone.c_str());
  }

  // Add the 'robot_available burger' predicate (no need to check or delete existing ones)
  plansys2::Predicate available_predicate;
  available_predicate.node_type = plansys2_msgs::msg::Node::PREDICATE;
  available_predicate.name = "robot_available";
  available_predicate.parameters.push_back(parser::pddl::fromStringParam("burger"));

  if (!problem_client_->addPredicate(available_predicate))
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not add predicate [robot_available burger]");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Added predicate [robot_available burger] to knowledge base");
  }
}




  void
  Scan::add_to_knowledge_base(const std::string &model_name, const geometry_msgs::msg::Pose &pose)
  {
    // Add the instance
    if (!problem_client_->addInstance(plansys2::Instance(model_name, "piece")))
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not add instance [%s]", model_name.c_str());
      return;
    }

    // Find the closest zone
    std::string closest_zone = find_closest_zone(pose.position);

    // Retrieve existing predicates
    auto predicates = problem_client_->getPredicates();

    // Search and remove existing predicates related to the models
    for (const auto &predicate : predicates)
    {
      // Check if the predicate is 'piece_at' and matches the model name
      if (predicate.name == "piece_at" && predicate.parameters.size() == 2)
      {
        if (predicate.parameters[0].name == model_name)
        {
          // Remove the existing predicate
          if (problem_client_->removePredicate(predicate))
          {
            RCLCPP_INFO(
                node_->get_logger(),
                "Removed existing predicate: %s %s %s",
                predicate.name.c_str(),
                predicate.parameters[0].name.c_str(),
                predicate.parameters[1].name.c_str());
          }
          else
          {
            RCLCPP_WARN(
                node_->get_logger(),
                "Failed to remove existing predicate: %s %s %s",
                predicate.name.c_str(),
                predicate.parameters[0].name.c_str(),
                predicate.parameters[1].name.c_str());
          }
        }
      }
    }

    // Add the predicate
    plansys2::Predicate predicate;
    predicate.node_type = plansys2_msgs::msg::Node::PREDICATE;
    predicate.name = "piece_at";
    predicate.parameters.push_back(parser::pddl::fromStringParam(model_name));
    predicate.parameters.push_back(parser::pddl::fromStringParam(closest_zone));

    if (!problem_client_->addPredicate(predicate))
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not add predicate [piece_at %s %s]", model_name.c_str(), closest_zone.c_str());
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Added predicate [piece_at %s %s] to knowledge base", model_name.c_str(), closest_zone.c_str());
    }
  }

  std::string
  Scan::find_closest_zone(const geometry_msgs::msg::Point &position)
  {
    std::string closest_zone;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto &zone : ZONES)
    {
      geometry_msgs::msg::Point zone_point;
      zone_point.x = zone.second[0];
      zone_point.y = zone.second[1];
      zone_point.z = zone.second[2];

      double distance = calculate_distance(position, zone_point);

      if (distance < min_distance)
      {
        min_distance = distance;
        closest_zone = zone.first;
      }
    }

    return closest_zone;
  }

} // namespace workshop_plansys2

// Register the node
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<workshop_plansys2::Scan>("Scan");
}
