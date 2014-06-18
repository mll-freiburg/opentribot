cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()
set( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

rosbuild_add_boost_directories()

rosbuild_add_executable(eposserver eposserver.cpp)
target_link_libraries(eposserver ntcan openrave-core)
rosbuild_link_boost(eposserver thread)

rosbuild_add_executable(testeposmotor testeposmotor.cpp)
target_link_libraries(testeposmotor ntcan openrave-core)
rosbuild_link_boost(testeposmotor thread)
