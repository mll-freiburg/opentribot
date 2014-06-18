TARGET = CalibrationRecorder
CONFIG = app warn_on debug qt
LIBS += dc1394_control
FORMS = CalibrationRecorderControlPanel.ui
QTHEADERS = ../../components/ImageWidget.h
SOURCES = \
main.cpp \
CalibrationRecorder.cpp \
../../../ImageProcessing/Calibration/GrayLevelImage.cc \
../../../ImageProcessing/Calibration/Convolution.cc \
../../../ImageProcessing/Calibration/edgeDetection.cc \
../../../ImageProcessing/Calibration/Pixelset.cc \
../../../ImageProcessing/Formation/RGBImage.cc \
../../../ImageProcessing/Formation/Image.cc \
../../../ImageProcessing/Formation/ImageBuffer.cc \
../../../ImageProcessing/Formation/IIDC.cc \
../../../ImageProcessing/Formation/ImageSourceFactory.cc \
../../../ImageProcessing/Formation/CamDriver.cc \
../../../ImageProcessing/Formation/ImageConversion.cc \
../../../ImageProcessing/Formation/Painter.cc \
../../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../../Structures/TribotsException.cc \
../../../Structures/Journal.cc \
../../../Fundamental/ConfigReader.cc \
../../../Fundamental/stringconvert.cc \
../../../Fundamental/Time.cc \
../../../Fundamental/Vec.cc \
../../../Fundamental/Angle.cc \
../../../Fundamental/geometry.cc \
../../../Fundamental/Frame2D.cc \
../../components/ImageWidget.cpp \
