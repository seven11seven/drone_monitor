cmake_minimum_required(VERSION 3.8)
project(gcopter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

include_directories(
    include
    # /usr/local/include/ompl-1.6
    # /opt/ros/humble/include
)

add_executable(global_planning src/global_planning.cpp)
ament_target_dependencies(global_planning 
    rclcpp 
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    OMPL
    ament_cmake
)
install(TARGETS
  global_planning
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

# add_executable(bls debug/bls.cpp)
# ament_target_dependencies(bls 
#     OMPL
# )

ament_package()