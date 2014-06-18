### TARGET: Dateiname der Zieldatei
TARGET = ../../../bin/colorTool


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS = 


### LIBS: programmspezifische Programmbibliotheken
LIBS = raw1394 dc1394_control jpeg AssiLib 


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = ../../Libs/jpeg-6b/ ../../Libs/Alib/AssiLib 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = ../../Libs/robotCtrLib/include ../../Libs/jpeg-6b ../../Libs/Alib/AssiLib


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = colorTool.ui


### QTHEADERS: Header, die QT-Makros enthalten
QTHEADERS = \
../components/ImageWidget.h \
../components/ScrollImageWidget.h \
CameraSettingsWidget.h \
../components/IDQObjects.h


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES	= main.cpp \
ColorTools.cpp \
CameraSettingsWidget.cpp \
LineFilterGui.cc \
CircleFilterGui.cc \
../components/IDQObjects.cpp \
GrabbingThread.cpp \
../components/ImageWidget.cpp \
../components/ScrollImageWidget.cpp \
../../ImageProcessing/PixelAnalysis/YUVLookupTable.cc \
../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../ImageProcessing/ObjectAnalysis/ColorClasses.cc \
../../ImageProcessing/ObjectAnalysis/LineFilter.cc \
../../ImageProcessing/ObjectAnalysis/CircleFilter.cc \
../../ImageProcessing/Formation/ImageIO.cc \
../../ImageProcessing/Formation/JPEGIO.cc \
../../ImageProcessing/Formation/BufferedIO.cc \
../../ImageProcessing/Formation/PPMIO.cc \
../../ImageProcessing/PixelAnalysis/Image2WorldMapping.cc \
../../ImageProcessing/PixelAnalysis/OmniCameraMapping.cc \
../../ImageProcessing/PixelAnalysis/DirectionalCameraMapping.cc \
../../ImageProcessing/Formation/ImageProducer.cc \
../../ImageProcessing/Formation/ImageBuffer.cc \
../../ImageProcessing/Formation/Image.cc \
../../ImageProcessing/PixelAnalysis/RobotMask.cc \
../../ImageProcessing/Formation/RGBImage.cc \
../../ImageProcessing/Formation/YUVImage.cc \
../../ImageProcessing/Formation/UYVYImage.cc \
../../ImageProcessing/Formation/PS3EyeImageSource.cc \
../../ImageProcessing/Formation/ImageConversion.cc \
../../ImageProcessing/Formation/FileSource.cc \
../../ImageProcessing/Formation/IIDC.cc \
../../ImageProcessing/Formation/CamDriver.cc \
../../ImageProcessing/Formation/FileMonitor.cc \
../../ImageProcessing/Formation/ImageSourceFactory.cc \
../../ImageProcessing/Formation/Painter.cc \
../../ImageProcessing/Calibration/CameraOptics.cc \
../../Structures/TribotsException.cc \
../../Structures/Journal.cc \
../../Fundamental/Angle.cc \
../../Fundamental/ConfigReader.cc \
../../Fundamental/Frame2D.cc \
../../Fundamental/Time.cc \
../../Fundamental/Vec.cc \
../../Fundamental/Vec3D.cc \
../../Fundamental/geometry.cc \
../../Fundamental/geometry3D.cc \
../../Fundamental/stringconvert.cc \
