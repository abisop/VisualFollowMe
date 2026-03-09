cmake_minimum_required(VERSION 3.8)
project(visual_follow_me)

# Default to C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#======================================================================================================

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

find_package(keyboard_msgs REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(vision_processing src/vision_processing_base.cpp)

ament_target_dependencies(vision_processing rclcpp sensor_msgs cv_bridge OpenCV)

install(TARGETS vision_processing
  DESTINATION lib/${PROJECT_NAME}
)

#install launch files 
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Visual Follow Me mode node (PX4 external mode)
find_package(px4_ros2_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(visual_follow_me_node 
    src/visual_follow_me.cpp
)
target_link_libraries(visual_follow_me_node
  Eigen3::Eigen
  px4_ros2_cpp::px4_ros2_cpp
  rclcpp::rclcpp
)

ament_target_dependencies(visual_follow_me_node rclcpp sensor_msgs cv_bridge OpenCV keyboard_msgs)

install(TARGETS visual_follow_me_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/path_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
