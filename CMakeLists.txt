cmake_minimum_required(VERSION 3.5)
project(lbr_servo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_common REQUIRED)
moveit_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        ament_cmake
        rclcpp
        rclcpp_action
        tf2_geometry_msgs
        tf2_ros
        moveit_core
        moveit_ros_planning_interface
        interactive_markers
        tf2_geometry_msgs
        moveit_ros_planning
        pluginlib
        Eigen3
        Boost
        control_msgs
        moveit_servo
        )

add_executable(servo_command src/servo_command.cpp)
target_include_directories(servo_command PUBLIC include)
ament_target_dependencies(servo_command ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
        TARGETS
        servo_command
        DESTINATION
        lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()
