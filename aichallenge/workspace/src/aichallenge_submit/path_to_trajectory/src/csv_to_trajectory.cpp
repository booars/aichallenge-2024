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

#include "path_to_trajectory/csv_to_trajectory.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>


CsvToTrajectory::CsvToTrajectory() : Node("csv_to_trajectory_node")
, cloud_(new pcl::PointCloud<pcl::PointXYZ>())
 {
  using std::placeholders::_1;
  this->declare_parameter<std::string>("csv_file_path", "");
  this->declare_parameter<float>("velocity_coef", 30.0f);
  this->declare_parameter<float>("trajectory_length", 100.0f);
  this->declare_parameter<float>("trajectory_margin", 2.0f);
  this->declare_parameter<float>("trajectory_rear_length", 10.0f);
  this->declare_parameter<float>("z_position", 0.0f);

  std::string csv_file_path;
  this->get_parameter("csv_file_path", csv_file_path);
  this->get_parameter("trajectory_length", this->trajectory_length_);
  this->get_parameter("z_position", this->z_position_);
  dynamicLoadParam();

  if (csv_file_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No CSV file path provided");
      rclcpp::shutdown();
      return;
  }
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "in_odom", qos, std::bind(&CsvToTrajectory::odomCallback, this, _1));
  this->pub_ = this->create_publisher<Trajectory>("output", 1);
  this->pub_now_point_ = this->create_publisher<std_msgs::msg::Int32>("now_waypoint", 1);
  this->readCsv(csv_file_path);

}

void CsvToTrajectory::dynamicLoadParam(){
  this->get_parameter("velocity_coef", this->velocity_coef_);
  this->get_parameter("trajectory_margin", this->trajectory_margin_);
  this->get_parameter("trajectory_rear_length", this->trajectory_rear_length_);
}

void CsvToTrajectory::readCsv(const std::string& file_path) {
  std::ifstream file(file_path);
  std::string line;
  double old_x=0.0;
  double old_y=0.0;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#' || line[0] == 'x') continue; // Skip empty lines and comments
    std::istringstream s(line);
    std::string field;
    std::vector<double> values;

    while (getline(s, field, ',')) {
      values.push_back(std::stod(field));
    }
    // x,y,z,yaw
    TrajectoryPoint point;
    point.pose.position.x = values[1];
    point.pose.position.y = values[2];
    point.pose.position.z = z_position_;
    //const double yaw = std::atan2(point.pose.position.y-old_y, point.pose.position.x-old_x);
    const double yaw = values[3];
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = sin(yaw / 2);
    point.pose.orientation.w = cos(yaw / 2);
    point.longitudinal_velocity_mps = values[5];
    point.acceleration_mps2 = 0.0; //values[6];

    trajectory_points_.push_back(point);
    old_x = point.pose.position.x;
    old_y = point.pose.position.y;
    cloud_->points.push_back(pcl::PointXYZ(point.pose.position.x, point.pose.position.y, z_position_));
  }
  // double yaw = std::atan2(trajectory_points_.front().pose.position.y-old_y, trajectory_points_.front().pose.position.x-old_x);
  // trajectory_points_.front().pose.orientation.x = 0.0;
  // trajectory_points_.front().pose.orientation.y = 0.0;
  // trajectory_points_.front().pose.orientation.z = sin(yaw / 2);
  // trajectory_points_.front().pose.orientation.w = cos(yaw / 2);
  RCLCPP_INFO(this->get_logger(), "Loaded %zu trajectory points", trajectory_points_.size());

  kdtree_.setInputCloud(cloud_);
  
}

void CsvToTrajectory::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odometry)
{
  if (current_point_index_ >= trajectory_points_.size()) return;

  dynamicLoadParam();

  Trajectory trajectory;
  // Set trajectory header
  trajectory.header = odometry->header;

  const int K = 1; // 1点のみ取得
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  pcl::PointXYZ searchPoint(odometry->pose.pose.position.x, odometry->pose.pose.position.y, z_position_);

  if (kdtree_.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
      // std::cout << "The closest point index: " << pointIdxNKNSearch[0] << std::endl;
      // std::cout << "Distance: " << pointNKNSquaredDistance[0] << std::endl;
  }
  else
  {
      // std::cout << "No neighbors found!" << std::endl;
      return;
  }
  int start_index = pointIdxNKNSearch[0]- std::round(trajectory_rear_length_/trajectory_margin_);
  if(start_index<0){
    start_index+=trajectory_points_.size();
  }
  const int points_num = std::round((trajectory_length_+trajectory_rear_length_)/trajectory_margin_);
  for(int i = 0; i < points_num; i++){
    int p = start_index + i;
    if(p>=trajectory_points_.size()){
      p-=(trajectory_points_.size());
    }
    trajectory.points.push_back(trajectory_points_[p]);
  }
  for(auto && p : trajectory.points){
    p.longitudinal_velocity_mps*=this->velocity_coef_;
  }
  pub_->publish(trajectory);
}

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CsvToTrajectory>());
  rclcpp::shutdown();
  return 0;
}
