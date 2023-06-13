cmake_minimum_required(VERSION 3.1)

project(ROS_codes)

add_executable(test_subscriber ROS_codes/test_subscriber.cpp)
target_link_libraries(test_subscriber ${catkin_LIBRARIES})

