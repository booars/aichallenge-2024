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

#ifndef COSTMAP_GENERATOR__COSTMAP_PARAMETERS_HPP_
#define COSTMAP_GENERATOR__COSTMAP_PARAMETERS_HPP_

#include <memory>

namespace costmap_generator
{
class CostmapParameters
{
public:
  using SharedPtr = std::shared_ptr<CostmapParameters>;

  static CostmapParameters::SharedPtr create_parameters(const double width, const double resolution)
  {
    return std::make_shared<CostmapParameters>(width, resolution);
  }

  explicit CostmapParameters(const double width, const double resolution)
  {
    width_ = width;
    width_2_ = width / 2.0;
    resolution_ = resolution;
    resolution_inv_ = 1.0 / resolution;
    grid_width_2_ = static_cast<int>(width * resolution_inv_) / 2;
    grid_width_ = grid_width_2_ * 2;
    grid_num_ = grid_width_ * grid_width_;
  }

private:
  double width_;
  double width_2_;
  double resolution_;
  double resolution_inv_;
  int grid_width_2_;
  int grid_width_;
  int grid_num_;
};
}  // namespace costmap_generator

#endif  // COSTMAP_GENERATOR__COSTMAP_PARAMETERS_HPP_
