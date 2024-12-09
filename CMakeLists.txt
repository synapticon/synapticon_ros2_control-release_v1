cmake_minimum_required(VERSION 3.8)
project(synapticon_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_library(
  synapticon_ros2_control
  SHARED
  src/synapticon_interface.cpp
)
target_compile_features(synapticon_ros2_control PUBLIC cxx_std_17)
target_include_directories(synapticon_ros2_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
ament_target_dependencies(
  synapticon_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface synapticon_ros2_control.xml)

install(
  DIRECTORY include/
  DESTINATION include/synapticon_ros2_control
)
install(TARGETS synapticon_ros2_control
  EXPORT export_synapticon_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # TODO
endif()

ament_export_targets(export_synapticon_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
