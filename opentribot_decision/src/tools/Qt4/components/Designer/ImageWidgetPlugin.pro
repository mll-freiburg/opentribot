TARGET = $(HOME)/.qt4/designer/libImageWidgetPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
ImageWidgetPlugin.h \
../ImageWidget.h \

SOURCES = \
ImageWidgetPlugin.cpp \
../ImageWidget.cpp \
../../../../ImageProcessing/Formation/Image.cc \
../../../../ImageProcessing/Formation/RGBImage.cc \
../../../../Fundamental/Time.cc \
../../../../Structures/TribotsException.cc \
../../../../ImageProcessing/Formation/ImageBuffer.cc \
../../../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
