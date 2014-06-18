TARGET = $(HOME)/.qt4/designer/libCameraSettingsWidgetPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
CameraSettingsWidgetPlugin.h \
../CameraSettingsWidget.h \
../../../components/IDQObjects.h \

SOURCES = \
CameraSettingsWidgetPlugin.cpp \
../CameraSettingsWidget.cpp \
../../../components/IDQObjects.cpp \
../../../../../ImageProcessing/Formation/IIDC.cc \
../../../../../ImageProcessing/Formation/CamDriver.cc \
../../../../../ImageProcessing/Formation/ImageBuffer.cc \
../../../../../ImageProcessing/Formation/ImageSourceFactory.cc \
../../../../../Fundamental/ConfigReader.cc \
../../../../../Fundamental/stringconvert.cc \
../../../../../Structures/TribotsException.cc \
