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

#add_custom_target(
#   LALIB
#   COMMAND make
#   WORKING_DIRECTORY ../src/Libs/LA/src
#)
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

INCLUDE_DIRECTORIES(src/Libs/LA/include  )

LINK_DIRECTORIES(../src/Libs/LA/lib  )

#set(all_src_fils src/decision
#		  src/Structures/TribotsException.cc )


rosbuild_add_executable(decision
src/decision.cc
src/Structures/TribotsException.cc 
src/Fundamental/ConfigReader.cc 
src/Fundamental/Time.cc 
src/Fundamental/Angle.cc 
src/Fundamental/Frame2D.cc 
src/Fundamental/Joystick.cc 
src/Fundamental/PIDController.cc 
src/Fundamental/VSController.cc 
src/Fundamental/PiecewiseLinearFunction.cc 
src/Fundamental/POSIXThread.cc 
src/Fundamental/Vec.cc 
src/Fundamental/Vec3D.cc 
src/Fundamental/binary_encoding.cc 
src/Fundamental/geometry.cc 
src/Fundamental/geometry3D.cc 
src/Fundamental/stringconvert.cc 
src/Fundamental/SmoothingFilter.cc 
src/Communication/UDPSocket.cc 
src/Fundamental/RemoteTune.cc 
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
src/WorldModel/Types/WorldModelTypeBase.cc 
src/WorldModel/Types/RosListenerWorldModel.cc 
src/Behavior/BDIBehavior.cc 
src/Behavior/SPBehavior.cc 
src/Behavior/RandomChoiceBehavior.cc 
src/Behavior/Behavior.cc 
src/Behavior/CycleCallBackRegistry.cc 
src/Behavior/Skill.cc 
src/Behavior/Behaviors/ApproachingBall/BApproachBallDirectly.cc 
src/Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingAwayOwnGoal.cc 
src/Behavior/Behaviors/ApproachingBall/BApproachBallForStandardSituation.cc 
src/Behavior/Behaviors/ApproachingBall/BApproachBallForOwnKickOff.cc 
src/Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingToGoal.cc 
src/Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingToMiddle.cc 
src/Behavior/Behaviors/ApproachingBall/BComplexApproachBallFreePlay.cc 
src/Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.cc 
src/Behavior/Behaviors/BallHandling/BWingAttack.cc 
src/Behavior/Behaviors/BallHandling/BPassSpontaneously.cpp 
src/Behavior/Behaviors/BallHandling/BQuerpassToSupport.cpp 
src/Behavior/Behaviors/BallHandling/BDribbleBallToGoal.cc 
src/Behavior/Behaviors/BallHandling/BEindhoven.cc 
src/Behavior/Behaviors/BallHandling/BBreakAttack.cc 
src/Behavior/Behaviors/BallHandling/BShakeOffDefender.cc 
src/Behavior/Behaviors/BallHandling/BRetreatDribble.cc 
src/Behavior/Behaviors/BallHandling/BDribbleBallToPassPosition.cc 
src/Behavior/Behaviors/BallHandling/BEigenMove.cc 
src/Behavior/Behaviors/BallHandling/BTouchBallAfterStandard.cc 
src/Behavior/Behaviors/BallHandling/BStuckOwnsBall.cc 
src/Behavior/Behaviors/BallHandling/BStuckDistanceShooter.cc 
src/Behavior/Behaviors/BallHandling/BBoostBallToGoal.cc 
src/Behavior/Behaviors/BasicMovements/BEmergencyStop.cc 
src/Behavior/Behaviors/BasicMovements/BFarShoot.cc 
src/Behavior/Behaviors/BasicMovements/BShoot.cc 
src/Behavior/Behaviors/BasicMovements/BShootEmergency.cc 
src/Behavior/Behaviors/BasicMovements/BShootImmediately.cc 
src/Behavior/Behaviors/BasicMovements/BDraufhalten.cc 
src/Behavior/Behaviors/BasicMovements/BStayInsideArea.cc 
src/Behavior/Behaviors/BasicMovements/BGotoPosEvadeObstacles.cpp 
src/Behavior/Behaviors/Goalie/BGoalieAttackBall.cc 
src/Behavior/Behaviors/Goalie/BGoalieBallInGoal.cc 
src/Behavior/Behaviors/Goalie/BGoalieFastPositioning.cc 
src/Behavior/Behaviors/Goalie/BGoalieFetchBall.cc 
src/Behavior/Behaviors/Goalie/BGoalieFetchBallLaterally.cc 
src/Behavior/Behaviors/Goalie/BGoalieFetchBallNearGoalPost.cc 
src/Behavior/Behaviors/Goalie/BGoalieGetAwayFromGoalPosts.cc 
src/Behavior/Behaviors/Goalie/BGoalieGetOutOfGoal.cc 
src/Behavior/Behaviors/Goalie/BGoalieOpponentGoalKick.cc 
src/Behavior/Behaviors/Goalie/BGoalieOpponentKickOff.cc 
src/Behavior/Behaviors/Goalie/BGoaliePatrol.cc 
src/Behavior/Behaviors/Goalie/BGoaliePenalty.cc 
src/Behavior/Behaviors/Goalie/BGoaliePositioning.cc 
src/Behavior/Behaviors/Goalie/BGoaliePositioningChipKick.cc 
src/Behavior/Behaviors/Goalie/BGoalieRaisedBall.cc 
src/Behavior/Behaviors/SpecialGameStates/BGameStopped.cc 
src/Behavior/Behaviors/SpecialGameStates/BPostOpponentStandardSituation.cc 
src/Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituation.cc 
src/Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituationNew.cc 
src/Behavior/Behaviors/SpecialGameStates/BPreOwnIndirectStandardSituation.cc 
src/Behavior/Behaviors/SpecialGameStates/BTestBehavior.cc 
src/Behavior/Behaviors/WithoutBall/BPatrolNoBall.cc 
src/Behavior/Behaviors/WithoutBall/BPreventInterferenceWithPass.cc 
src/Behavior/Behaviors/WithoutBall/BSupportLongPass.cc 
src/Behavior/Behaviors/WithoutBall/BSupportNearBall.cc 
src/Behavior/Behaviors/ZonePressure/BZonePressure.cc 
src/Behavior/Behaviors/ZonePressure/BDefendBall.cc 
src/Behavior/Behaviors/ZonePressure/BDoubleTeam.cc 
src/Behavior/Behaviors/ZonePressure/BSupportDoubleTeamSideline.cc 
src/Behavior/Behaviors/ZonePressure/BProtectGoal.cc 
src/Behavior/Behaviors/ZonePressure/BSafety.cc 
src/Behavior/Behaviors/ZonePressure/BSupportDoubleTeamMiddle.cc 
src/Behavior/Skills/WithoutBall/SPatrol.cc 
src/Behavior/Behaviors/WithoutBall/BBlockWayToGoal.cc 
src/Behavior/Behaviors/WithoutBall/BBlockWayToMiddle.cc 
src/Behavior/Behaviors/WithoutBall/BBlockWayToOpponentGoal.cc 
src/Behavior/Behaviors/WithoutBall/BAvoidGoalieArea.cc 
src/Behavior/Predicates/RadialObstacleView.cc 
src/Behavior/Skills/ApproachingBall/SApproachMovingBall.cc 
src/Behavior/Skills/ApproachingBall/SApproachParkedBall.cc 
src/Behavior/Skills/BallHandling/SDribbleBallStraightToPosEvadeSidewards.cc 
src/Behavior/Skills/BallHandling/SDribbleBallToPos.cc 
src/Behavior/Skills/BasicMovements/SBoostToPos.cc 
src/Behavior/Skills/BasicMovements/SGoToPosEvadeObstacles.cc 
src/Behavior/Skills/BasicMovements/SGoToPosEvadeObstaclesOld.cc 
src/Behavior/Skills/Goalie/SPhysFollowBall.cc 
src/Behavior/Skills/Goalie/SPhysGotoBallCarefully.cc 
src/Behavior/Skills/Goalie/SPhysGotoPos.cc 
src/Behavior/Skills/Goalie/SPhysGotoPosAvoidObstacles.cc 
src/Behavior/Skills/Goalie/SPhysGotoPosViaTurningPoint.cc 
src/Behavior/Skills/Goalie/SPhysTurnAroundPos.cc 
src/Behavior/Skills/Goalie/SPhysVolley.cc 
src/Behavior/Behaviors/SpecialGameStates/BOwnPenalty.cc 
src/Behavior/Behaviors/ApproachingBall/BInterceptBall.cc 
src/Behavior/Predicates/freeCorridor.cc 
src/Behavior/Behaviors/BallHandling/BBefreiungsschlag.cc 
src/Behavior/Behaviors/BallHandling/BPass.cpp 
src/Behavior/Behaviors/WithoutBall/BCounterAttack.cc 
src/Behavior/Behaviors/ApproachingBall/BVolleyApproach.cc 
src/Behavior/Behaviors/WithoutBall/BLeaveGoal.cc 
src/Behavior/Behaviors/ApproachingBall/BApproachBallAfterNonexecutedStandard.cc 
src/Behavior/Behaviors/SpecialGameStates/BTestStateStop.cc 
src/Behavior/Behaviors/Setup/BTestStateStuckStop.cc 
src/Behavior/Behaviors/Setup/BTestOwnsBallCheck.cc 
src/Behavior/Behaviors/Setup/BTestAllDirections.cc 
src/Behavior/Behaviors/ApproachingBall/BInterceptBallStatic.cc 
src/Behavior/Behaviors/ApproachingBall/BCatchBall.cc 
src/Behavior/Behaviors/ApproachingBall/BTurnAroundPos.cc 
src/Behavior/Behaviors/ApproachingBall/BApproachBallStatic.cc 
src/Behavior/Behaviors/WithoutBall/BOpposeBall.cc 
src/Behavior/Skills/Goalie/SPhysGotoBallAvoidObstacles.cc 
src/Behavior/Skills/BallHandling/SPass.cc 
src/Behavior/Behaviors/BallHandling/BPassBeforeGoal.cc 
src/Behavior/Behaviors/Setup/BNewDribbleTest.cc 
src/Behavior/Behaviors/Setup/BSecurity.cc 
src/Player/Player.cc 
src/Player/PlayerTipkick.cc 
src/Player/PassPlayer.cc 
src/Player/HeikoPlayer.cc 
src/Player/SetupPlayer.cc 
src/Player/AddGotoPosPlayer.cc 
src/Player/AddJoystickPlayer.cc 
src/Player/BehaviorPlayer.cc 
src/Player/SimpleDefender.cc 
src/Player/FieldPlayer07.cc 
src/Player/HumanCoopPlayer.cc 
src/Player/Goalie.cc 
src/Player/JoystickPlayer.cc 
src/Player/JoystickPlayerUDP.cc 
src/Player/MultiRolePlayer.cc 
src/Player/PlayerDummy.cc 
src/Player/PlayerFactory.cc 
src/Player/SingleRolePlayer.cc 
src/Player/TutorialPlayer.cc 
src/Player/ZickZackTestPlayer.cc 
src/Player/SaschasPassPlayer.cc 
src/Player/WhiteBoard.cc 
src/Player/DribbleTestPlayer.cc 
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


target_link_libraries(decision LA )

add_dependencies(decision LALIB)

