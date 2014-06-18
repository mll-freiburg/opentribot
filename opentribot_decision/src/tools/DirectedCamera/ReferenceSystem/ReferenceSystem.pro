### TARGET: Dateiname der Zieldatei
TARGET = ReferenceSystem


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS =


### LIBS: programmspezifische Programmbibliotheken
LIBS = raw1394 dc1394_control jpeg GLG3D_debug GL G3D_debug GLU z SDL gsl gslcblas


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = ../../Libs/jpeg-6b/ 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = ../../Libs/jpeg-6b /usr/local/lib/g3d-6_07/include/ /usr/include/SDL/


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = ControlWidget.ui

### QTHEADERS: Header, die QT-Makros enthalten
QTHEADERS = \
../../components/ImageWidget.h \
ReferenceSystem.h

### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES += main.cpp \
	   Visualization.cpp \
	   ReferenceSystem.cpp \
	   OpenGLApp.cpp \
           ../../components/ImageWidget.cpp \
           ../../../ImageProcessing/Formation/ImageProducer.cc \
           ../../../ImageProcessing/Formation/MultiImageProducer.cc \
           ../../../ImageProcessing/Formation/Image.cc \
           ../../../ImageProcessing/Formation/RGBImage.cc \
           ../../../ImageProcessing/Formation/YUVImage.cc \
           ../../../ImageProcessing/Formation/YUV411Image.cc \
           ../../../Fundamental/ConfigReader.cc \
           ../../../ImageProcessing/PixelAnalysis/Image2WorldMapping.cc \
           ../../../ImageProcessing/PixelAnalysis/World2ImageMapping.cc \
           ../../../ImageProcessing/PixelAnalysis/Image2WorldMapping3D.cc \
           ../../../Fundamental/Vec.cc \
           ../../../Fundamental/geometry3D.cc \
           ../../../Fundamental/geometry3Dgsl.cc \
           ../../../ImageProcessing/Formation/ImageBuffer.cc \
           ../../../Fundamental/Time.cc \
           ../../../Structures/TribotsException.cc \
           ../../../Fundamental/geometry.cc \
           ../../../Fundamental/Angle.cc \
           ../../../Fundamental/Frame2D.cc \
           ../../../Fundamental/Vec3D.cc \
           ../../../ImageProcessing/Formation/IIDC.cc \
           ../../../ImageProcessing/Formation/FileSource.cc \
           ../../../ImageProcessing/Formation/UYVYImage.cc \
           ../../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
           ../../../ImageProcessing/PixelAnalysis/YUVLookupTable.cc \
           ../../../ImageProcessing/Formation/ImageIO.cc \
           ../../../ImageProcessing/Formation/FileMonitor.cc \
           ../../../ImageProcessing/Formation/JPEGIO.cc \
           ../../../ImageProcessing/Formation/PPMIO.cc \
           ../../../ImageProcessing/Formation/ImageConversion.cc \
           ../../../Structures/Journal.cc \
           ../../../ImageProcessing/PixelAnalysis/RobotMask.cc \
           ../../../ImageProcessing/Formation/Painter.cc \
           ../../../Fundamental/stringconvert.cc \
           ../../../ImageProcessing/ObjectAnalysis/ColorClasses.cc
