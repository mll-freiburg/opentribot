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
   LALIB
   COMMAND make
   WORKING_DIRECTORY ../src/Libs/LA/src
)
#target_link_libraries(myexecutable full_path_to_generated_library)









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
#target_link_libraries(example ${PROJECT_NAME})

INCLUDE_DIRECTORIES(
src/Libs/LA/include 

)

LINK_DIRECTORIES(
../src/Libs/LA/lib 
)



rosbuild_add_executable(worldmodel
src/worldmodel.cc
src/Structures/TribotsException.cc
src/Fundamental/ConfigReader.cc 
src/Fundamental/Time.cc 
src/Fundamental/Angle.cc 
src/Fundamental/Frame2D.cc 
src/Fundamental/Vec.cc 
src/Fundamental/Vec3D.cc 
src/Fundamental/binary_encoding.cc 
src/Fundamental/geometry.cc 
src/Fundamental/geometry3D.cc 
src/Fundamental/stringconvert.cc 
src/Fundamental/SmoothingFilter.cc 
src/Structures/BallLocationReadWriter.cc 
src/Structures/DriveVector.cc 
src/Structures/DriveVectorReadWriter.cc 
src/Structures/FieldGeometry.cc 
src/Structures/GameState.cc 
src/Structures/GameStateReadWriter.cc 
src/Structures/GyroDataReadWriter.cc 
src/Structures/Journal.cc 
src/Structures/MessageBoard.cc 
src/Structures/MessageBoardReadWriter.cc 
src/Structures/ObstacleLocation.cc 
src/Structures/ObstacleLocationReadWriter.cc 
src/Structures/RobotData.cc 
src/Structures/RobotLocationReadWriter.cc 
src/Structures/RobotProperties.cc 
src/Structures/TacticsBoard.cc 
src/Structures/TacticsBoardDescription.cc 
src/Structures/TeammateLocation.cc 
src/Structures/VisibleObject.cc 
src/Structures/VisibleObjectReadWriter.cc 
src/Structures/default_exception_handler.cc 
src/WorldModel/WorldModel.cc 
src/WorldModel/WorldModelFactory.cc 
src/WorldModel/Ball/DynamicSlidingWindowBallFilter3D.cc 
src/WorldModel/Ball/MultiCameraBallFilter.cc 
src/WorldModel/Prediction/LocationShortTermMemory.cc 
src/WorldModel/Prediction/ObjectInteractionManager.cc 
src/WorldModel/Prediction/SingleStepHeuristicVelocityPredictor.cc 
src/WorldModel/Prediction/update_robot_location.cc 
src/WorldModel/Obstacles/EMAObstacleFilter.cc 
src/WorldModel/Obstacles/ObstacleContainer.cc 
src/WorldModel/Orga/GameStateManager.cc 
src/WorldModel/Orga/OdometryContainer.cc 
src/WorldModel/Orga/RefereeStateMachine.cc 
src/WorldModel/Orga/VisualContainer.cc 
src/WorldModel/SL/FieldLUT.cc 
src/WorldModel/SL/RobotPositionKalmanFilter.cc 
src/WorldModel/SL/RobotPositionPredictor.cc 
src/WorldModel/SL/SLStuckSensor.cc 
src/WorldModel/SL/SLVelocitySensor.cc 
src/WorldModel/SL/VisualPositionOptimiser.cc 
src/WorldModel/SL/TemporalDifferenceSL.cc 
src/WorldModel/Types/AddWriteWorldModel.cc 
src/WorldModel/Types/ErrorMinimiserWorldModel.cc 
src/WorldModel/Types/FileWorldModel.cc 
src/WorldModel/Types/WorldModelDummy.cc 
src/WorldModel/Types/WorldModelTypeBase.cc
src/UserInterface/UserInterface.cc 
src/UserInterface/AddComUserInterface/AddComUserInterface.cc 
src/UserInterface/CliUserInterface/CliUserInterface.cc 
src/UserInterface/StreamUserInterface/StreamUserInterface.cc 
src/Communication/MultiPacketUDPCommunication.cc 
src/Communication/PriorityUDPCommunication.cc 
src/Communication/TaggedUDPCommunication.cc 
src/Communication/NonspecificTaggedUDPCommunication.cc 
src/Communication/TribotsUDPCommunication.cc 
src/Communication/UDPSocket.cc 
src/Communication/encoding.cc 
)


target_link_libraries(worldmodel LA )

add_dependencies(worldmodel LALIB)


