#ifndef WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__SCAN_HPP_
#define WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__SCAN_HPP_

#include <string>
#include <unordered_map>
#include <vector>
#include <optional>  // Include for std::optional
#include <memory>  // Include for std::shared_ptr

#include "rclcpp/rclcpp.hpp"  // Include ROS 2 client library
#include "gazebo_msgs/srv/get_model_list.hpp"  // Include the service type
#include "gazebo_msgs/srv/get_entity_state.hpp"  // Include the GetEntityState

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"  // Include for Pose messages

#include "plansys2_problem_expert/ProblemExpertClient.hpp"  // Include for ProblemExpertClient

namespace workshop_plansys2
{

class Scan : public BT::ActionNodeBase
{
public:
  explicit Scan(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  int counter_;
  rclcpp::Node::SharedPtr node_;  // Node pointer for ROS 2 communication
  rclcpp::Client<gazebo_msgs::srv::GetModelList>::SharedPtr get_model_list_client_;  // Service client for GetModelList
  rclcpp::Client<gazebo_msgs::srv::GetEntityState>::SharedPtr get_entity_state_client_;  // Client for GetEntityState
   
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;  // PlanSys2 ProblemExpertClient

  // Utility functions
  std::optional<geometry_msgs::msg::Pose> get_position(const std::string &model_name);
  double calculate_distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2);

  // Function to add models to the knowledge base
  void add_to_knowledge_base(const std::string &model_name, const geometry_msgs::msg::Pose &pose);
  void add_robot_to_nearest_zone(const geometry_msgs::msg::Pose &pose);
  void delete_zone_predicates(const geometry_msgs::msg::Pose &robot_pose);

  // Function to find the closest zone to a given position
  std::string find_closest_zone(const geometry_msgs::msg::Point &position);
};

}  // namespace workshop_plansys2

#endif  // WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__SCAN_HPP_
