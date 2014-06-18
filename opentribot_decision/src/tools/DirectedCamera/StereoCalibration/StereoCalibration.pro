CONFIG = app qt warn_on debug

FORMS = \
StereoCalibration.ui \

QTHEADERS = \
../../../tools/components/ImageWidget.h \
../../../tools/components/ScrollImageWidget.h \

SOURCES = \
main.cpp \
../../../ImageProcessing/Formation/RGBImage.cc \
../../../ImageProcessing/Formation/Image.cc \
../../../ImageProcessing/Formation/ImageBuffer.cc \
../../../ImageProcessing/Formation/ImageConversion.cc \
../../../ImageProcessing/Formation/Painter.cc \
../../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../../ImageProcessing/Formation/PPMIO.cc \
../../../ImageProcessing/Formation/ImageIO.cc \
../../../ImageProcessing/PixelAnalysis/OmniCameraMapping.cc \
../../../ImageProcessing/PixelAnalysis/Image2WorldMapping.cc \
../../../Structures/TribotsException.cc \
../../../Fundamental/ConfigReader.cc \
../../../Fundamental/stringconvert.cc \
../../../Fundamental/Time.cc \
../../../Fundamental/Vec.cc \
../../../Fundamental/Vec3D.cc \
../../../Fundamental/Angle.cc \
../../../Fundamental/geometry.cc \
../../../Fundamental/geometry3D.cc \
../../../Fundamental/Frame2D.cc \
../../../Structures/Journal.cc \
../../../tools/components/ImageWidget.cpp \
../../../tools/components/ScrollImageWidget.cpp \
