cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

INCLUDE_DIRECTORIES(src/bvCan src/kicker src/udp_imu/usbimu/)

#LINK_DIRECTORIES(/home/tribot/ALib/AssiLib ) 


rosbuild_add_executable(motionserver src/motionserver.cpp 
	src/Time.cc
	src/bvCan/bvcan.cpp 
        src/bvCan/bvcanbuseposprotocol1.cpp 
        src/bvCan/bvcanopenmessagehandler.cpp 
        src/bvCan/bvcanopenmessageset.cpp 
        src/bvCan/bvcanopenobject.cpp 
        src/bvCan/bvfd.cpp 
        src/bvCan/bvglobal.cpp 
        src/bvCan/bvmessagebase.cpp 
)


target_link_libraries(motionserver pcan usb)
	
ADD_DEFINITIONS(-DUSE_CAN)
	
