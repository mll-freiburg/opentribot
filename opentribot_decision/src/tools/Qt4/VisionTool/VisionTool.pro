#LIBS += LA dc1394_control raw1394 jpeg

#libs for osx/libdc2
LIBS += -lLA -ldc1394_control -ljpeg

LIBPATH += ../../../Libs/LA/lib ../../../Libs/jpeg-6b 
INCLUDEPATH += ../../../Libs/LA/include
CONFIG = warn_on debug qt
RESOURCES = VisionTool.qrc
FORMS = \
Widgets/UI/ImageMaskWidget.ui \
Widgets/UI/ImageCenterWidget.ui \
Widgets/UI/BalanceAreaWidget.ui \
Widgets/UI/CameraDefinitionWidget.ui \
Widgets/UI/SettingsWidget.ui \
Widgets/UI/ColorCalibrationWidget.ui \

QTHEADERS = \
Widgets/VisionToolMainWindow.h \
Widgets/ImageMaskWidget.h \
Widgets/ImageCenterWidget.h \
Widgets/BalanceAreaWidget.h \
Widgets/VisionToolWidget.h \
Widgets/CameraDefinitionWidget.h \
Widgets/CameraSettingsWidget.h \
Widgets/SettingsWidget.h \
Widgets/ColorCalibrationWidget.h \
../components/DoubleSlider.h \
../components/ImageWidget.h \
../components/ScrollImageWidget.h \
../components/IDQObjects.h \

SOURCES = \
Logic/main.cpp \
Logic/VisionToolImageSource.cpp \
Widgets/VisionToolMainWindow.cpp \
Widgets/ImageMaskWidget.cpp \
Widgets/ImageCenterWidget.cpp \
Widgets/BalanceAreaWidget.cpp \
Widgets/CameraDefinitionWidget.cpp \
Widgets/CameraSettingsWidget.cpp \
Widgets/SettingsWidget.cpp \
Widgets/ColorCalibrationWidget.cpp \
../components/ImageWidget.cpp \
../components/DoubleSlider.cpp \
../components/ScrollImageWidget.cpp \
../components/IDQObjects.cpp \
../components/MorphologicOperators.cpp \
../../../ImageProcessing/Calibration/ImageArea.cc \
../../../ImageProcessing/Calibration/centerRingOperation.cc \
../../../ImageProcessing/Formation/RGBImage.cc \
../../../ImageProcessing/Formation/IIDC.cc \
../../../ImageProcessing/Formation/CamDriver.cc \
../../../ImageProcessing/Formation/FileSource.cc \
../../../ImageProcessing/Formation/Image.cc \
../../../ImageProcessing/Formation/ImageBuffer.cc \
../../../ImageProcessing/Formation/ImageSourceFactory.cc \
../../../ImageProcessing/Formation/PPMIO.cc \
../../../ImageProcessing/Formation/JPEGIO.cc \
../../../ImageProcessing/Formation/ImageIO.cc \
../../../ImageProcessing/Formation/ImageConversion.cc \
../../../ImageProcessing/Formation/Painter.cc \
../../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../../ImageProcessing/PixelAnalysis/RobotMask.cc \
../../../ImageProcessing/PixelAnalysis/YUVLookupTable.cc \
../../../ImageProcessing/ObjectAnalysis/ChainCoding.cc \
../../../ImageProcessing/ObjectAnalysis/Regions.cc \
../../../ImageProcessing/ObjectAnalysis/ColorClasses.cc \
../../../Structures/TribotsException.cc \
../../../Structures/Journal.cc \
../../../Fundamental/Time.cc \
../../../Fundamental/Vec.cc \
../../../Fundamental/Angle.cc \
../../../Fundamental/ConfigReader.cc \
../../../Fundamental/stringconvert.cc \
Logic/GrabbingThread.cpp \
../../../Fundamental/POSIXThread.cc \
Logic/ColorTools.cpp \
