### TARGET: Dateiname der Zieldatei
TARGET = ../../../bin/ImageMask


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS =


### LIBS: programmspezifische Programmbibliotheken
LIBS = raw1394 dc1394_control jpeg RobotCtr2 pcan pthread 


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = ../../Libs/RobotCtr2/lib ../../Libs/jpeg-6b/ 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = ../../Libs/RobotCtr2/include ../../Libs/jpeg-6b


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = imagemaskmainwidget.ui


### QTHEADERS: Header, die QT-Makros enthalten
QTHEADERS = ../components/ImageWidget.h


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES	= main.cpp \
../components/ImageWidget.cc \
../components/ImageVarianceFilter.cpp \
../components/MorphologicOperators.cpp \
../../Robot/Robot.cc \
../../Robot/RobotDummy.cc \
../../Robot/FileRobot.cc \
../../Robot/OmniRobot_Tribot_CAN.cc \
../../Robot/RobotFactory.cc \
../../Robot/CompassGrabbingThread.cc \
../../WorldModel/WorldModel.cc \
../../WorldModel/WorldModelFactory.cc \
../../WorldModel/Types/WorldModelDummy.cc \
../../ImageProcessing/Formation/Image.cc \
../../ImageProcessing/Formation/IIDC.cc \
../../ImageProcessing/Formation/CamDriver.cc \
../../ImageProcessing/Formation/FileSource.cc \
../../ImageProcessing/Formation/UYVYImage.cc \
../../ImageProcessing/Formation/RGBImage.cc \
../../ImageProcessing/Formation/YUVImage.cc \
../../ImageProcessing/Formation/ImageConversion.cc \
../../ImageProcessing/Formation/ImageBuffer.cc \
../../ImageProcessing/Formation/ImageIO.cc \
../../ImageProcessing/Formation/ImageSourceFactory.cc \
../../ImageProcessing/Formation/JPEGIO.cc \
../../ImageProcessing/Formation/Painter.cc \
../../ImageProcessing/Formation/PPMIO.cc \
../../ImageProcessing/ObjectAnalysis/ChainCoding.cc \
../../ImageProcessing/ObjectAnalysis/LineScanning.cc \
../../ImageProcessing/ObjectAnalysis/LineFilter.cc \
../../ImageProcessing/ObjectAnalysis/ScanLines.cc \
../../ImageProcessing/Formation/ImageProducer.cc \
../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../ImageProcessing/PixelAnalysis/Image2WorldMapping.cc \
../../ImageProcessing/PixelAnalysis/DirectionalCameraMapping.cc \
../../ImageProcessing/PixelAnalysis/OmniCameraMapping.cc \
../../ImageProcessing/ObjectAnalysis/ColorClasses.cc \
../../ImageProcessing/PixelAnalysis/YUVLookupTable.cc \
../../ImageProcessing/Formation/FileMonitor.cc \
../../ImageProcessing/PixelAnalysis/RobotMask.cc \
../../ImageProcessing/ObjectAnalysis/Regions.cc \
../../ImageProcessing/Calibration/CameraOptics.cc \
../../Fundamental/ConfigReader.cc \
../../Fundamental/Vec.cc \
../../Fundamental/Vec3D.cc \
../../Fundamental/geometry.cc \
../../Fundamental/geometry3D.cc \
../../Fundamental/Angle.cc \
../../Fundamental/Time.cc \
../../Fundamental/stringconvert.cc \
../../Fundamental/SmoothingFilter.cc \
../../Structures/TribotsException.cc  \
../../Structures/DriveVector.cc \
../../Structures/FieldGeometry.cc \
../../Structures/GameState.cc \
../../Structures/Journal.cc \
../../Structures/ObstacleLocation.cc \
../../Structures/RobotProperties.cc \
../../Structures/RobotData.cc \
../../Structures/VisibleObject.cc \
../../Structures/DriveVectorReadWriter.cc \
../../Structures/GyroDataReadWriter.cc \
../../Structures/MessageBoard.cc \
../../Structures/default_exception_handler.cc \
../../Fundamental/Frame2D.cc \
../../Fundamental/POSIXThread.cc \
../../Fundamental/binary_encoding.cc \
../../ImageProcessing/Formation/BufferedIO.cc \
