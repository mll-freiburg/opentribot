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

INCLUDE_DIRECTORIES(src/Libs/jpeg-6b src/Libs/LA/include)

LINK_DIRECTORIES(../src/Libs/jpeg-6b  ../src/Libs/LA/lib )





add_definitions(-g )

rosbuild_add_executable(visionserver src/visionserver.cc
src/ImageProcessing/Vision.cc 
src/Structures/TribotsException.cc 
src/Fundamental/ConfigReader.cc 
src/Fundamental/Time.cc 
src/ImageProcessing/Types/ScanLineImageProcessing.cc 
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
src/ImageProcessing/ImageProcessing.cc 
src/ImageProcessing/ImageProcessingFactory.cc 
src/ImageProcessing/VisionFactory.cc 
src/ImageProcessing/Calibration/CameraOptics.cc 
src/ImageProcessing/Formation/FileMonitor.cc 
src/ImageProcessing/Formation/FileSource.cc 
src/ImageProcessing/Formation/Image.cc 
src/ImageProcessing/Formation/ImageBuffer.cc 
src/ImageProcessing/Formation/ImageConversion.cc 
src/ImageProcessing/Formation/ImageIO.cc 
src/ImageProcessing/Formation/BufferedIO.cc 
src/ImageProcessing/Formation/ImageProducer.cc 
src/ImageProcessing/Formation/ImageSourceFactory.cc 
src/ImageProcessing/Formation/JPEGIO.cc 
src/ImageProcessing/Formation/MultiImageProducer.cc 
src/ImageProcessing/Formation/Painter.cc 
src/ImageProcessing/Formation/PPMIO.cc 
src/ImageProcessing/Formation/RGBImage.cc 
src/ImageProcessing/Formation/UYVYImage.cc 
src/ImageProcessing/Formation/YUVImage.cc 
src/ImageProcessing/Formation/YUV411Image.cc 
src/ImageProcessing/ObjectAnalysis/BallDetector.cc 
src/ImageProcessing/ObjectAnalysis/BallLearner.cc
src/ImageProcessing/ObjectAnalysis/ChainCoding.cc 
src/ImageProcessing/ObjectAnalysis/ColorClasses.cc 
src/ImageProcessing/ObjectAnalysis/ContourObstacleDetector.cc 
src/ImageProcessing/ObjectAnalysis/FieldMapper.cc 
src/ImageProcessing/ObjectAnalysis/LineDetector.cc 
src/ImageProcessing/ObjectAnalysis/LineScanning.cc 
src/ImageProcessing/ObjectAnalysis/LineFilter.cc 
src/ImageProcessing/ObjectAnalysis/Regions.cc 
src/ImageProcessing/ObjectAnalysis/ScanLines.cc 
src/ImageProcessing/ObjectAnalysis/SimpleCluster.cc 
src/ImageProcessing/ObjectAnalysis/SimpleObstacleDetector.cc 
src/ImageProcessing/PixelAnalysis/ColorClassifier.cc 
src/ImageProcessing/PixelAnalysis/Image2WorldMapping.cc 
src/ImageProcessing/PixelAnalysis/OmniCameraMapping.cc 
src/ImageProcessing/PixelAnalysis/DirectionalCameraMapping.cc 
src/ImageProcessing/PixelAnalysis/RobotMask.cc 
src/ImageProcessing/PixelAnalysis/YUVLookupTable.cc
src/ImageProcessing/PixelAnalysis/YUVLookupLearner.cc
src/ImageProcessing/Types/FileVision.cc 
src/ImageProcessing/Types/VisionDummy.cc 
src/ImageProcessing/Types/GenericMultiSourceVision.cc 
src/ImageProcessing/Types/SimpleVision.cc
src/Structures/BallLocationReadWriter.cc 
src/Structures/DriveVector.cc 
src/Structures/DriveVectorReadWriter.cc 
src/Structures/FieldGeometry.cc 
src/Structures/GameState.cc 
src/Structures/GameStateReadWriter.cc 
src/Structures/GyroDataReadWriter.cc 
src/Structures/Journal.cc 
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
src/ImageProcessing/ObjectAnalysis/PolarObstacleCluster.cc 
src/ImageProcessing/ObjectAnalysis/PolarObstacleDetector.cc 
src/ImageProcessing/Calibration/centerRingOperation.cc 
src/ImageProcessing/Calibration/ImageArea.cc 
src/ImageProcessing/Calibration/ImageCenterSurveillance.cc 
src/Audio/WavePlay.cc 
src/ImageProcessing/Formation/PS3EyeImageSource.cc
#src/ImageProcessing/Formation/IIDC.cc 
#src/ImageProcessing/Formation/CamDriver.cc 
src/tools/DistanceCalibration/DistanceCalibration.cc 
src/tools/DistanceCalibration/ImageProcessingDistanceCalibration.cc



)
target_link_libraries(visionserver jpeg  LA stdc++ fltk fltk_images)

#add_dependencies(visionserver LALIB)


