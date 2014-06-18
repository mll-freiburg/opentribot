### TARGET: Dateiname der Zieldatei
TARGET = ../../../bin/CalibrationTool

### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release

LIBS = raw1394 dc1394_control jpeg LA AssiLib
LIBPATH = ../../Libs/RobotCtr2/lib ../../Libs/jpeg-6b/ ../../Libs/LA/lib ../../Libs/Alib/AssiLib/
INCLUDEPATH = ../../Libs/RobotCtr2/include ../../Libs/jpeg-6b ../../Libs/LA/include 
INCLUDEPATH += ../../Libs/Alib/AssiLib

FORMS = DistanceCalibrationCollectionWidget.ui
QTHEADERS = ../components/ImageWidget.h ../components/ScrollImageWidget.h


SOURCES = \
main.cpp \
RGBColorClassifier.cpp \
DistanceCalibrationImageAnalysis.cpp \
ImageMaskBuilder.cpp \
DistMarkerBuilder.cpp \
../components/ImageWidget.cpp \
../components/ScrollImageWidget.cpp \
../../Fundamental/ConfigReader.cc \
../../Fundamental/Angle.cc \
../../Fundamental/Vec.cc \
../../Fundamental/Time.cc \
../../Fundamental/stringconvert.cc \
../../Fundamental/POSIXThread.cc \
../../Fundamental/SmoothingFilter.cc \
../../Structures/Journal.cc\
../../Structures/TribotsException.cc \
../../Structures/DriveVector.cc \
../../Structures/RobotProperties.cc \
../../Structures/RobotData.cc \
../../ImageProcessing/Formation/ImageBuffer.cc \
../../ImageProcessing/Formation/Image.cc \
../../ImageProcessing/Formation/YUVImage.cc \
../../ImageProcessing/Formation/RGBImage.cc \
../../ImageProcessing/Formation/FileSource.cc \
../../ImageProcessing/Formation/ImageSourceFactory.cc \
../../ImageProcessing/Formation/ImageIO.cc \
../../ImageProcessing/Formation/JPEGIO.cc \
../../ImageProcessing/Formation/PPMIO.cc \
../../ImageProcessing/Formation/ImageConversion.cc \
../../ImageProcessing/Formation/Painter.cc \
../../ImageProcessing/Formation/IIDC.cc \
../../ImageProcessing/Formation/PS3EyeImageSource.cc \
../../ImageProcessing/PixelAnalysis/RobotMask.cc \
../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../Robot/Robot.cc \
../../Robot/RobotFactory.cc \
../../WorldModel/WorldModel.cc \
../../WorldModel/WorldModelFactory.cc \
../../ImageProcessing/Formation/CamDriver.cc \
../../WorldModel/Types/WorldModelDummy.cc \
../../Structures/ObstacleLocation.cc \
../../Structures/GameState.cc \
../../Structures/MessageBoard.cc \
../../Structures/FieldGeometry.cc \
../../Fundamental/geometry.cc \
../../Fundamental/Frame2D.cc \
../../ImageProcessing/Calibration/centerRingOperation.cc \
../../ImageProcessing/Calibration/ImageArea.cc \

LIBS += RobotCtr2 pcan pthread
SOURCES += ../../Robot/OmniRobot_Tribot_CAN.cc \
../../Robot/OmniRobot_MotionClient.cc \
../../Communication/UDPSocket.cc \
../../Robot/OnlineRobotMonitorServer.cc \
../../Robot/CompassGrabbingThread.cc \
