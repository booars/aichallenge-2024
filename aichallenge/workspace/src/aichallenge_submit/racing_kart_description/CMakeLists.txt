cmake_minimum_required(VERSION 3.5)
project(racing_kart_description)

add_compile_options(-std=c++14)

find_package(ament_cmake_auto REQUIRED)
find_package(xacro REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_package(INSTALL_TO_SHARE
  mesh
  urdf
  config
)
