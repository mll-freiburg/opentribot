TARGET = testShutterRing
LIBS += LA
LIBPATH += ../../Libs/LA/lib
INCLUDEPATH += ../../Libs/LA/include
CONFIG = warn_on release
SOURCES = \
main.cc \
../../ImageProcessing/Calibration/ImageArea.cc \
../../ImageProcessing/Calibration/centerRingOperation.cc \
../../ImageProcessing/Formation/RGBImage.cc \
../../ImageProcessing/Formation/Image.cc \
../../ImageProcessing/Formation/ImageBuffer.cc \
../../ImageProcessing/Formation/PPMIO.cc \
../../ImageProcessing/Formation/ImageIO.cc \
../../ImageProcessing/Formation/ImageConversion.cc \
../../ImageProcessing/Formation/Painter.cc \
../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../Structures/TribotsException.cc \
../../Fundamental/Time.cc \
../../Fundamental/Vec.cc \
../../Fundamental/Angle.cc \
