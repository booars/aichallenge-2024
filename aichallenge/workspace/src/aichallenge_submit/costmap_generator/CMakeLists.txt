cmake_minimum_required(VERSION 3.8)
project(costmap_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(global_costmap_generator SHARED
  src/global_costmap_generator.cpp
)

rclcpp_components_register_node(global_costmap_generator
  PLUGIN "costmap_generator::GlobalCostmapGenerator"
  EXECUTABLE global_costmap_generator_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)