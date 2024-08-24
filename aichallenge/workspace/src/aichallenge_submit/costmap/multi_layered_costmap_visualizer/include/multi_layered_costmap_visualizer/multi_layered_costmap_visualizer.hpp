// Copyright 2024 Booars
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

#ifndef MULTI_LAYERED_COSTMAP_VISUALIZER__MULTI_LAYERED_COSTMAP_VISUALIZER_HPP_
#define MULTI_LAYERED_COSTMAP_VISUALIZER__MULTI_LAYERED_COSTMAP_VISUALIZER_HPP_

#include <booars_costmap_utils/booars_costmap_utils.hpp>
#include <multi_layered_costmap/multi_layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace multi_layered_costmap_visualizer
{

using MultiLayeredCostmap = multi_layered_costmap::MultiLayeredCostmap;

class MultiLayeredCostmapVisualizer : public rclcpp::Node
{
public:
  explicit MultiLayeredCostmapVisualizer(const rclcpp::NodeOptions & options);

private:
  MultiLayeredCostmap::SharedPtr multi_layered_costmap_;
};
};  // namespace multi_layered_costmap_visualizer

#endif  // MULTI_LAYERED_COSTMAP_VISUALIZER__MULTI_LAYERED_COSTMAP_VISUALIZER_HPP_
