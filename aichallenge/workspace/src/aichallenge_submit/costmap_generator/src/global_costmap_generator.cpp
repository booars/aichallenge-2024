// Copyright 2024 Fool Stuck Engineers
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

#include "costmap_generator/global_costmap_generator.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

namespace costmap_generator
{
GlobalCostmapGenerator::GlobalCostmapGenerator(const rclcpp::NodeOptions & options)
: Node("global_costmap_generator", options)
{
  // Declare parameters
  double update_rate;
  {
    update_rate = this->declare_parameter("update_rate", 20.0);
  }

  // Create subscriptions
  {
    map_sub_ = this->create_subscription<HADMapBin>(
      "~/input/map", 1,
      std::bind(&GlobalCostmapGenerator::map_callback, this, std::placeholders::_1));
  }

  // Create publishers
  {
    costmap_pub_ = this->create_publisher<OccupancyGrid>("~/output/costmap", 1);
  }

  // Create function timers
  {
    update_timer_ = FunctionTimer::create_function_timer(
      this, update_rate, std::bind(&GlobalCostmapGenerator::update, this));
  }
}

void GlobalCostmapGenerator::update()
{
  // Check if map is available
  if (!map_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "No map received yet");
    return;
  }
}

void GlobalCostmapGenerator::map_callback(const HADMapBin::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Received map");
  map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map_);
}

}  // namespace costmap_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_generator::GlobalCostmapGenerator)
