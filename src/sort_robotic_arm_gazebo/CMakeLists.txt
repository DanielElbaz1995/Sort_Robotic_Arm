cmake_minimum_required(VERSION 3.8)
project(sort_robotic_arm_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sort_robotic_arm_interface REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_executable(arm_controller_node src/arm_controller.cpp)
ament_target_dependencies(arm_controller_node rclcpp rclcpp_action sort_robotic_arm_interface trajectory_msgs )
add_executable(object_detector_node src/object_detector.cpp)
ament_target_dependencies(object_detector_node rclcpp rclcpp_action sensor_msgs cv_bridge sort_robotic_arm_interface)
add_executable(IK_calculation_node src/IK_calculation_old.cpp)
ament_target_dependencies(IK_calculation_node rclcpp rclcpp_action sort_robotic_arm_interface trajectory_msgs)

install(TARGETS 
        arm_controller_node
        object_detector_node
        IK_calculation_node
        DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch world src include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
