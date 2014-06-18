TARGET = $(HOME)/.qt4/designer/libScrollImageWidgetPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
ScrollImageWidgetPlugin.h \
../ImageWidget.h \
../ScrollImageWidget.h \

SOURCES = \
ScrollImageWidgetPlugin.cpp \
../ImageWidget.cpp \
../ScrollImageWidget.cpp \
../../../../ImageProcessing/Formation/Image.cc \
../../../../ImageProcessing/Formation/RGBImage.cc \
../../../../Fundamental/Time.cc \
../../../../Structures/TribotsException.cc \
../../../../ImageProcessing/Formation/ImageBuffer.cc \
../../../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
