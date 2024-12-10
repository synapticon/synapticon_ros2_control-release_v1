cmake_minimum_required(VERSION 3.8)
project(synapticon_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()

# SOEM library
set(OS "linux")
set(OS_LIBS pthread rt)
file(GLOB SOEM_SOURCES soem/*.c)
file(GLOB OSAL_SOURCES osal/${OS}/*.c)
file(GLOB OSHW_SOURCES oshw/${OS}/*.c)

file(GLOB SOEM_HEADERS soem/*.h)
file(GLOB OSAL_HEADERS osal/osal.h osal/${OS}/*.h)
file(GLOB OSHW_HEADERS oshw/${OS}/*.h)

add_library(soem STATIC
  ${SOEM_SOURCES}
  ${OSAL_SOURCES}
  ${OSHW_SOURCES}
  ${OSHW_EXTRA_SOURCES})
target_link_libraries(soem ${OS_LIBS})
target_include_directories(soem PUBLIC
  osal
  osal/${OS}
  oshw/${OS}
  soem
)

# ros2_control plugin library
add_library(
  synapticon_ros2_control
  SHARED
  src/soem_utilities.cpp
  src/synapticon_interface.cpp
)
target_compile_features(synapticon_ros2_control PUBLIC cxx_std_17)
target_include_directories(synapticon_ros2_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/osal>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/osal/${OS}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/oshw/${OS}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/soem>
)
ament_target_dependencies(
  synapticon_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface synapticon_ros2_control.xml)

# ethercat_grant
add_executable(ethercat_grant ethercat_grant/src/ethercat_grant.cpp)
target_link_libraries(ethercat_grant cap)
ament_target_dependencies(ethercat_grant
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include/synapticon_ros2_control
)
install(
  TARGETS ethercat_grant synapticon_ros2_control
  EXPORT export_synapticon_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES
  ${SOEM_HEADERS}
  ${OSAL_HEADERS}
  ${OSHW_HEADERS}
  DESTINATION include/synapticon_ros2_control
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/synapticon_ros2_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/synapticon_ros2_control
)

if(BUILD_TESTING)
  # TODO
endif()

ament_export_targets(export_synapticon_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
