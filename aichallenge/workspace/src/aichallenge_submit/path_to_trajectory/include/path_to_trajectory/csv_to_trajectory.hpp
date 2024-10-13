// Copyright 2023 Tier IV, Inc. All rights reserved.
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

#ifndef PATH_TO_TRAJECTORY__CSV_TO_TRAJECTORY_HPP_
#define PATH_TO_TRAJECTORY__CSV_TO_TRAJECTORY_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

class CsvToTrajectory : public rclcpp::Node {
public:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

  CsvToTrajectory();

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_now_point_;
  std::vector<TrajectoryPoint> trajectory_points_;
  size_t current_point_index_ = 0;
  float velocity_rate_ = 1.0f;
  float trajectory_length_ = 200.0f;
  float trajectory_margin_ = 2.0f;
  float next_point_threshold_ = 30.0f;
  int now_index_ = 0;
  float z_position_ = 0.0f;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;


  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry);
  void readCsv(const std::string& csv_file_path);
};

#endif  // PATH_TO_TRAJECTORY__CSV_TO_TRAJECTORY_HPP_