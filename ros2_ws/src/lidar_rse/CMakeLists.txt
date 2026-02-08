cmake_minimum_required(VERSION 3.8)
project(lidar_rse)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
include_directories(${apriltag_INCLUDE_DIRS})

# EXECUTABLES HERE
###################################################################

# gimbal_ctrl_node
add_executable(lidar_rse_node
	src/lidar_rse_node.cpp
    src/lidar_rse.cpp
    # src/ovnav_percept_server.cpp
)
ament_target_dependencies(lidar_rse_node
	rclcpp 
	geometry_msgs
    nav_msgs
	sensor_msgs
	mavros_msgs
	visualization_msgs
	message_filters
    cv_bridge
)
target_link_libraries(lidar_rse_node
	${OpenCV_LIBS}
    ${APRILTAG_LIBRARIES}
    ${PCL_LIBRARIES}
)
target_include_directories(lidar_rse_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
)

install(TARGETS lidar_rse_node DESTINATION lib/${PROJECT_NAME})


###################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()