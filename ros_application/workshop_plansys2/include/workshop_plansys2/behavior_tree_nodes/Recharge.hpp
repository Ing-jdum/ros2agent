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

#ifndef WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__RECHARGE_HPP_
#define WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__RECHARGE_HPP_

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace workshop_plansys2
{

class Recharge : public BT::ActionNodeBase
{
public:
  explicit Recharge(
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
};

}  // namespace workshop_plansys2

#endif  // WORKSHOP_PLANSYS2__BEHAVIOR_TREE_NODES__RECHARGE_HPP_
