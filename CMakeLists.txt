cmake_minimum_required(VERSION 3.5)
project(brownian_motion)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

###########
## Build ##
###########

add_executable(brownian_motion src/brownian_motion.cpp)
ament_target_dependencies(brownian_motion rclcpp sensor_msgs geometry_msgs)

# The following node does exactly the same than 'brownian_motion'
# But it is written in C language, without using C++ classes
add_executable(c_brownian_motion src/c_brownian_motion.cpp)
ament_target_dependencies(c_brownian_motion rclcpp sensor_msgs geometry_msgs)

#############
## Install ##
#############

install(
    TARGETS
        brownian_motion
    DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS
        c_brownian_motion
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
