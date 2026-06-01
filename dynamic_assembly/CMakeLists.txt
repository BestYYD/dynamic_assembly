cmake_minimum_required(VERSION 3.8)
project(dynamic_assembly)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(osqp REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  ros2_aruco_interfaces
  tf2_geometry_msgs     
  moveit_core           
  moveit_ros_planning   
  moveit_ros_planning_interface
)

# EKF Filter Node
add_library(ekf_filter_component SHARED src/ekf_filter_node.cpp)
target_include_directories(ekf_filter_component PUBLIC include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(ekf_filter_component ${DEPENDENCIES})
rclcpp_components_register_node(ekf_filter_component PLUGIN "dynamic_assembly::EkfFilterNode" EXECUTABLE ekf_filter_node)

# APF Planner Node
add_library(apf_planner_component SHARED src/apf_planner_node.cpp)
target_include_directories(apf_planner_component PUBLIC include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(apf_planner_component ${DEPENDENCIES})
rclcpp_components_register_node(apf_planner_component PLUGIN "dynamic_assembly::ApfPlannerNode" EXECUTABLE apf_planner_node)

# MPC Planner Node
add_library(mpc_planner_component SHARED src/mpc_planner_node.cpp)
target_include_directories(mpc_planner_component PUBLIC include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(mpc_planner_component ${DEPENDENCIES})
target_link_libraries(mpc_planner_component osqp::osqp)
rclcpp_components_register_node(mpc_planner_component PLUGIN "dynamic_assembly::MpcPlannerNode" EXECUTABLE mpc_planner_node)

# Hardware Interface Node
add_library(hardware_interface_component SHARED src/hardware_interface_node.cpp)
target_include_directories(hardware_interface_component PUBLIC include ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(hardware_interface_component ${DEPENDENCIES})
rclcpp_components_register_node(hardware_interface_component PLUGIN "dynamic_assembly::HardwareInterfaceNode" EXECUTABLE hardware_interface_node)

# main.cpp
add_executable(assembly_system src/main.cpp)
target_include_directories(assembly_system PUBLIC include ${EIGEN3_INCLUDE_DIR})

# 与 ament_target_dependencies一样，这里使用target_link_libraries是为了区分ROS2库和自己创建的库
target_link_libraries(assembly_system
  ekf_filter_component
  apf_planner_component
  mpc_planner_component
)
ament_target_dependencies(assembly_system ${DEPENDENCIES})

install(TARGETS
  ekf_filter_component
  apf_planner_component
  mpc_planner_component
  hardware_interface_component
  assembly_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
