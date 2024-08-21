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
#include <lanelet2_extension/utility/query.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <lanelet2_core/geometry/Polygon.h>

namespace costmap_generator
{
GlobalCostmapGenerator::GlobalCostmapGenerator(const rclcpp::NodeOptions & options)
: Node("global_costmap_generator", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  tf_broadcaster_(this)
{
  // Declare parameters
  double update_rate;
  {
    update_rate = this->declare_parameter("update_rate", 20.0);
    costmap_target_frame_id_ = this->declare_parameter("costmap_target_frame_id", "base_link");
    costmap_frame_id_ = this->declare_parameter("costmap_frame_id", "costmap");

    double costmap_width = this->declare_parameter("costmap_width", 20.0);
    double costmap_resolution = this->declare_parameter("costmap_resolution", 0.1);
    costmap_parameters_ = CostmapParameters::create_parameters(costmap_width, costmap_resolution);
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
  geometry_msgs::msg::TransformStamped target_transform;
  try {
    target_transform =
      tf_buffer_.lookupTransform("map", costmap_target_frame_id_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Could not get transform %s", ex.what());
    return;
  }

  // Cache the current position of the costmap center
  Vector3 costmap_center_position = target_transform.transform.translation;

  // Get the intersected lanelets
  lanelet::ConstLanelets intersected_lanelets = get_intersected_lanelets(costmap_center_position);

  // Create the costmap
  OccupancyGrid costmap;
  {
    costmap.info.width = costmap_parameters_->grid_width();
    costmap.info.height = costmap_parameters_->grid_width();
    costmap.info.resolution = costmap_parameters_->resolution();
    costmap.info.origin.position.x = costmap_center_position.x - costmap_parameters_->width_2();
    costmap.info.origin.position.y = costmap_center_position.y - costmap_parameters_->width_2();
    costmap.info.origin.position.z = 0.0;
    costmap.info.origin.orientation.x = 0.0;
    costmap.info.origin.orientation.y = 0.0;
    costmap.info.origin.orientation.z = 0.0;
    costmap.info.origin.orientation.w = 1.0;
  }

  // Fill the costmap
  {
    costmap.data.resize(costmap_parameters_->grid_num(), 100);
    for (int i = 0; i < costmap_parameters_->grid_num(); i++) {
      Point2d cell_position = get_cell_position(costmap_center_position, i);
      for (const auto & lanelet : intersected_lanelets) {
        if (boost::geometry::within(cell_position, lanelet.polygon2d().basicPolygon())) {
          costmap.data[i] = 0;
          break;
        }
      }
    }
  }

  // Publish the costmap TF
  {
    geometry_msgs::msg::TransformStamped costmap_transform = target_transform;
    costmap_transform.header.stamp = now();
    costmap_transform.child_frame_id = costmap_frame_id_;
    tf_broadcaster_.sendTransform(costmap_transform);
  }

  // Publish the costmap
  {
    costmap.header.stamp = now();
    costmap.header.frame_id = "map";
    costmap_pub_->publish(costmap);
  }
}

lanelet::ConstLanelets GlobalCostmapGenerator::get_intersected_lanelets(const Vector3 & center)
{
  LinearRing2d costmap_contour = get_costmap_contour(center);
  lanelet::ConstLanelets intersected_lanelets;
  for (const auto & road : roads_) {
    if (boost::geometry::intersects(costmap_contour, road.polygon2d().basicPolygon())) {
      intersected_lanelets.push_back(road);
    }
  }
  return intersected_lanelets;
}

tier4_autoware_utils::LinearRing2d GlobalCostmapGenerator::get_costmap_contour(
  const Vector3 & center)
{
  LinearRing2d costmap_contour;
  costmap_contour.push_back(
    {center.x - costmap_parameters_->width_2(), center.y - costmap_parameters_->width_2()});
  costmap_contour.push_back(
    {center.x + costmap_parameters_->width_2(), center.y - costmap_parameters_->width_2()});
  costmap_contour.push_back(
    {center.x + costmap_parameters_->width_2(), center.y + costmap_parameters_->width_2()});
  costmap_contour.push_back(
    {center.x - costmap_parameters_->width_2(), center.y + costmap_parameters_->width_2()});
  costmap_contour.push_back(
    {center.x - costmap_parameters_->width_2(), center.y - costmap_parameters_->width_2()});
  return costmap_contour;
}

tier4_autoware_utils::Point2d GlobalCostmapGenerator::get_cell_position(
  const Vector3 & costmap_origin, const int & index)
{
  const int x = index % costmap_parameters_->grid_width();
  const int y = index / costmap_parameters_->grid_width();
  return {
    costmap_origin.x + x * costmap_parameters_->resolution() - costmap_parameters_->width_2(),
    costmap_origin.y + y * costmap_parameters_->resolution() - costmap_parameters_->width_2()};
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
