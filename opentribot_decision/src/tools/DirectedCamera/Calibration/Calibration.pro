TARGET = Calibration
CONFIG = app warn_on release qt
INCLUDEPATH = 
LIBPATH = ../../../Libs/joeg-6b
LIBS = opencv jpeg   # ggf opencv statt cv
QTHEADERS = ../../../tools/components/ImageWidget.h
SOURCES = \
DirectionalCalibration.cc \
main.cc \
../../../ImageProcessing/Calibration/CameraOptics.cc \
../../../tools/components/ImageWidget.cpp \
../../../ImageProcessing/Formation/Image.cc \
../../../ImageProcessing/Formation/RGBImage.cc \
../../../ImageProcessing/Formation/YUVImage.cc \
../../../ImageProcessing/Formation/PPMIO.cc \
../../../ImageProcessing/PixelAnalysis/RobotMask.cc \
../../../Fundamental/ConfigReader.cc \
../../../Fundamental/Vec.cc \
../../../Fundamental/Vec3D.cc \
../../../Fundamental/stringconvert.cc \
../../../ImageProcessing/PixelAnalysis/Image2WorldMapping.cc \
../../../ImageProcessing/Formation/ImageBuffer.cc \
../../../Fundamental/Time.cc \
../../../Structures/TribotsException.cc \
../../../Structures/Journal.cc \
../../../Fundamental/Angle.cc \
../../../Fundamental/geometry.cc \
../../../Fundamental/Frame2D.cc \
../../../Fundamental/geometry3D.cc \
../../../ImageProcessing/Formation/UYVYImage.cc \
../../../ImageProcessing/Formation/YUV411Image.cc \
../../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../../ImageProcessing/PixelAnalysis/YUVLookupTable.cc \
../../../ImageProcessing/Formation/ImageIO.cc \
../../../ImageProcessing/Formation/FileMonitor.cc \
../../../ImageProcessing/Formation/JPEGIO.cc \
../../../ImageProcessing/Formation/ImageConversion.cc
