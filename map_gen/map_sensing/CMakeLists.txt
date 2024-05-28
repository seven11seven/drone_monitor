cmake_minimum_required(VERSION 3.8)
project(map_sensing)

#####################################
## Set g++ and complier parameters ##
#####################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#######################
## Find package path ##
#######################
# find dependencies
find_package(ament_cmake REQUIRED)
## External library
## CMAKE will search for the header file and related cmake file
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io) # Add the necessary PCL components
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(pcl_ros REQUIRED)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mockamap.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)
# add_executable(${PROJECT_NAME}_node
#     ${${PROJECT_NAME}_SRCS})
## Or you can declare the target executable separetly
add_executable(${PROJECT_NAME}_node src/forest_sensing.cpp)

## Add ament_cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
ament_target_dependencies(${PROJECT_NAME}_node 
    rclcpp 
    sensor_msgs
    pcl_conversions
    nav_msgs
    # pcl_ros
    ament_cmake
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  pcl_common
  pcl_kdtree
  flann
)

# target_include_directories(forest_sensing PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(forest_sensing PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
install(TARGETS 
${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
