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


add_custom_target(
   odelibs
   COMMAND make
   WORKING_DIRECTORY ../ode/ode-0.5/
)




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

INCLUDE_DIRECTORIES(. src)

LINK_DIRECTORIES(/usr/X11R6/lib ../ode/ode-0.5/lib ) 


rosbuild_add_executable(simulator src/Simulator/main.cpp 
src/Simulator/Communication.cpp 
src/Simulator/World.cpp 
src/Simulator/dynamics.cpp 
src/Simulator/external.cpp 
src/Simulator/helpers.cpp 
src/Communication/SimulatorUDPCommunication.cpp 
src/Communication/NonspecificTaggedUDPCommunication.cc 
src/Communication/TaggedUDPCommunication.cc 
src/Communication/PriorityUDPCommunication.cc 
src/Communication/MultiPacketUDPCommunication.cc 
src/Communication/UDPSocket.cc 
src/Fundamental/Time.cc 
)


target_link_libraries(simulator drawstuff ode X11 GL GLU pthread)
	
add_dependencies(simulator odelibs)
