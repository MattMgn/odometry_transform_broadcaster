cmake_minimum_required(VERSION 2.8.3)
project(odometry_transform_broadcaster)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs tf sensor_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(broadcast src/broadcast.cpp)
target_link_libraries(broadcast ${catkin_LIBRARIES})

add_executable(broadcast_imu src/broadcast_imu.cpp)
target_link_libraries(broadcast_imu ${catkin_LIBRARIES})
