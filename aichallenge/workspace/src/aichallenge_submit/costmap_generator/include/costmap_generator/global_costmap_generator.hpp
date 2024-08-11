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

#ifndef COSTMAP_GENERATOR__GLOBAL_COSTMAP_GENERATOR_HPP_
#define COSTMAP_GENERATOR__GLOBAL_COSTMAP_GENERATOR_HPP_

#include <booars_utils/ros/function_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <lanelet2_core/LaneletMap.h>

namespace costmap_generator
{

class GlobalCostmapGenerator : public rclcpp::Node
{
  using FunctionTimer = booars_utils::ros::FunctionTimer;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using HADMapBinSubscription = rclcpp::Subscription<HADMapBin>;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using OccupancyGridPublisher = rclcpp::Publisher<OccupancyGrid>;

public:
  explicit GlobalCostmapGenerator(const rclcpp::NodeOptions & options);

private:
  void update();
  void map_callback(const HADMapBin::SharedPtr msg);

  FunctionTimer::SharedPtr update_timer_;

  HADMapBinSubscription::SharedPtr map_sub_;
  lanelet::LaneletMapPtr map_;

  OccupancyGridPublisher::SharedPtr costmap_pub_;
};
}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__GLOBAL_COSTMAP_GENERATOR_HPP_
