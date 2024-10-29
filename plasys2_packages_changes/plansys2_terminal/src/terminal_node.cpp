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


#include <memory>
#include <signal.h>
#include "rclcpp/rclcpp.hpp"
#include "plansys2_terminal/Terminal.hpp"

// Function to ignore SIGPIPE signals
void ignore_sigpipe()
{
  struct sigaction sa;
  sa.sa_handler = SIG_IGN;  // Ignore SIGPIPE
  sigaction(SIGPIPE, &sa, nullptr);
}

int main(int argc, char ** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Ignore broken pipe (SIGPIPE) to prevent node shutdown due to broken pipes
  ignore_sigpipe();

  try
  {
    // Create terminal node
    auto terminal_node = std::make_shared<plansys2_terminal::Terminal>();

    // Run the terminal console
    terminal_node->run_console();

    // Shutdown ROS 2
    rclcpp::shutdown();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unhandled exception: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown error occurred, shutting down.");
    rclcpp::shutdown();
    return 1;
  }

  return 0;
}

