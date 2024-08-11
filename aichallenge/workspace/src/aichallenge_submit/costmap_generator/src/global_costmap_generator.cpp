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

namespace costmap_generator
{
GlobalCostmapGenerator::GlobalCostmapGenerator(const rclcpp::NodeOptions & options)
: Node("global_costmap_generator", options)
{
  had_map_bin_sub_ = this->create_subscription<HADMapBin>(
    "~/input/map", 1,
    std::bind(&GlobalCostmapGenerator::map_callback, this, std::placeholders::_1));
}

void GlobalCostmapGenerator::map_callback(const HADMapBin::SharedPtr msg)
{
  map_ = msg;
}

}  // namespace costmap_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_generator::GlobalCostmapGenerator)
