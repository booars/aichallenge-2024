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

#include <booars_utils/nav/occupancy_grid_utils.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>

namespace costmap_generator
{

GlobalCostmapGenerator::GlobalCostmapGenerator(const rclcpp::NodeOptions & options)
: Node("global_costmap_generator", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  // Declare parameters
  double update_rate;
  {
    update_rate = this->declare_parameter("update_rate", 20.0);
    map_frame_ = this->declare_parameter("map_frame_id", "map");
    target_frame_ = this->declare_parameter("costmap_frame_id", "base_link");
  }

  // Declare costmap parameters and setup costmap
  {
    double costmap_width = this->declare_parameter("costmap_width", 20.0);
    double costmap_resolution = this->declare_parameter("costmap_resolution", 0.1);
    costmap_parameters_ =
      OccupancyGridParameters::create_parameters(costmap_width, costmap_resolution);
    costmap_ = booars_utils::nav::occupancy_grid_utils::create_occupancy_grid(costmap_parameters_);
    costmap_->header.frame_id = map_frame_;
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

  // Get the transform from the map frame to the costmap target frame
  rclcpp::Time costmap_frame_time;
  Vector3 costmap_center_position;
  {
    geometry_msgs::msg::TransformStamped target_transform;
    try {
      target_transform = tf_buffer_.lookupTransform(map_frame_, target_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Could not get transform %s", ex.what());
      return;
    }
    costmap_frame_time = target_transform.header.stamp;
    costmap_center_position = target_transform.transform.translation;
  }

  // Get the intersected lanelets
  lanelet::ConstLanelets intersected_lanelets = get_intersected_lanelets(costmap_center_position);

  // Fill the costmap
  {
    for (int i = 0; i < costmap_parameters_->grid_num(); i++) {
      Point2d cell_position = this->get_global_cell_position(costmap_center_position, i);
      for (const auto & lanelet : intersected_lanelets) {
        if (boost::geometry::within(cell_position, lanelet.polygon2d().basicPolygon())) {
          costmap_->data[i] = 0;
          break;
        }
        costmap_->data[i] = 100;
      }
    }
  }

  // Update the costmap origin
  {
    booars_utils::nav::occupancy_grid_utils::update_origin(
      costmap_, costmap_parameters_, costmap_center_position);
  }

  // Publish the costmap
  {
    costmap_->header.stamp = costmap_frame_time;
    costmap_pub_->publish(*costmap_);
  }
}

lanelet::ConstLanelets GlobalCostmapGenerator::get_intersected_lanelets(
  const Vector3 & costmap_center)
{
  LinearRing2d costmap_contour = get_costmap_contour(costmap_center);
  lanelet::ConstLanelets intersected_lanelets;
  for (const auto & road : roads_) {
    if (boost::geometry::intersects(costmap_contour, road.polygon2d().basicPolygon())) {
      intersected_lanelets.push_back(road);
    }
  }
  return intersected_lanelets;
}

tier4_autoware_utils::LinearRing2d GlobalCostmapGenerator::get_costmap_contour(
  const Vector3 & costmap_center)
{
  LinearRing2d costmap_contour;
  costmap_contour.push_back(
    {costmap_center.x - costmap_parameters_->width_2(),
     costmap_center.y - costmap_parameters_->width_2()});
  costmap_contour.push_back(
    {costmap_center.x + costmap_parameters_->width_2(),
     costmap_center.y - costmap_parameters_->width_2()});
  costmap_contour.push_back(
    {costmap_center.x + costmap_parameters_->width_2(),
     costmap_center.y + costmap_parameters_->width_2()});
  costmap_contour.push_back(
    {costmap_center.x - costmap_parameters_->width_2(),
     costmap_center.y + costmap_parameters_->width_2()});
  costmap_contour.push_back(
    {costmap_center.x - costmap_parameters_->width_2(),
     costmap_center.y - costmap_parameters_->width_2()});
  return costmap_contour;
}

tier4_autoware_utils::Point2d GlobalCostmapGenerator::get_global_cell_position(
  const Vector3 & costmap_center, const int & index)
{
  auto local_cell_position =
    booars_utils::nav::occupancy_grid_utils::index_to_point(costmap_parameters_, index);
  return {costmap_center.x + local_cell_position(0), costmap_center.y + local_cell_position(1)};
}

void GlobalCostmapGenerator::map_callback(const HADMapBin::SharedPtr msg)
{
  map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, map_);
  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(map_);
  roads_ = lanelet::utils::query::roadLanelets(all_lanelets);
}

}  // namespace costmap_generator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap_generator::GlobalCostmapGenerator)
