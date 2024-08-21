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

#include "booars_utils/nav/occupancy_grid_parameters.hpp"

#include <booars_utils/ros/function_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace costmap_generator
{

class GlobalCostmapGenerator : public rclcpp::Node
{
  using FunctionTimer = booars_utils::ros::FunctionTimer;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using HADMapBinSubscription = rclcpp::Subscription<HADMapBin>;
  using LinearRing2d = tier4_autoware_utils::LinearRing2d;
  using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
  using OccupancyGridPublisher = rclcpp::Publisher<OccupancyGrid>;
  using Point2d = tier4_autoware_utils::Point2d;
  using Vector3 = geometry_msgs::msg::Vector3;
  using CostmapParameters = booars_utils::nav::OccupancyGridParameters;

public:
  explicit GlobalCostmapGenerator(const rclcpp::NodeOptions & options);

private:
  void update();
  lanelet::ConstLanelets get_intersected_lanelets(const Vector3 & center);
  LinearRing2d get_costmap_contour(const Vector3 & center);
  Point2d get_cell_position(const Vector3 & costmap_origin, const int & index);

  void map_callback(const HADMapBin::SharedPtr msg);

  FunctionTimer::SharedPtr update_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  HADMapBinSubscription::SharedPtr map_sub_;
  lanelet::LaneletMapPtr map_;
  lanelet::ConstLanelets roads_;

  OccupancyGridPublisher::SharedPtr costmap_pub_;

  std::string costmap_target_frame_id_;
  std::string costmap_frame_id_;
  CostmapParameters::SharedPtr costmap_parameters_;
};
}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__GLOBAL_COSTMAP_GENERATOR_HPP_
