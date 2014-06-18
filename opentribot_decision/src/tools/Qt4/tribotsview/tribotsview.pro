### TARGET: Dateiname der Zieldatei
TARGET = ../../../../bin/tribotsview


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS = 


### LIBS: programmspezifische Programmbibliotheken
LIBS = jpeg 


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = ../../../Libs/jpeg-6b/ 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = ../../../Libs/jpeg-6b


### RESOURCES: QT-Ressource-Datei (maximal eine)
RESOURCES = tribotsview.qrc

### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = \
UI/ImageviewWidget.ui \
UI/TribotsviewMainWidget.ui


### QTHEADERS: Header, die QT-Makros enthalten
QTHEADERS = \
../components/FieldOfPlay.h \
../components/ImageWidget.h \
../components/MonoLED.h \
RotateView.h \
SLErrorWidget.h \
ImageviewWidget.h \
TribotsviewMainWidget.h


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES	= \
main.cpp \
CycleContainer.cpp \
RotateView.cpp \
SLErrorWidget.cpp \
ImageviewWidget.cpp \
TribotsviewMainWidget.cpp \
../components/MonoLED.cpp \
../components/CycleInfo.cpp \
../components/FieldOfPlay.cpp \
../components/ImageWidget.cc \
../../../Structures/ObstacleLocation.cc \
../../../Structures/VisibleObject.cc \
../../../Structures/DriveVector.cc \
../../../Structures/FieldGeometry.cc \
../../../Structures/GameState.cc \
../../../Structures/TribotsException.cc \
../../../Structures/RobotLocationReadWriter.cc \
../../../Structures/BallLocationReadWriter.cc \
../../../Structures/ObstacleLocationReadWriter.cc \
../../../Structures/VisibleObjectReadWriter.cc \
../../../Structures/GameStateReadWriter.cc \
../../../Structures/DriveVectorReadWriter.cc \
../../../Fundamental/Angle.cc \
../../../Fundamental/Vec.cc \
../../../Fundamental/Vec3D.cc \
../../../Fundamental/Time.cc \
../../../Fundamental/stringconvert.cc \
../../../Fundamental/binary_encoding.cc \
../../../Fundamental/geometry.cc \
../../../Fundamental/ConfigReader.cc \
../../../Fundamental/Frame2D.cc \
../../../WorldModel/SL/FieldLUT.cc \
../../../WorldModel/SL/VisualPositionOptimiser.cc \
../../../WorldModel/Orga/VisualContainer.cc \
../../../ImageProcessing/Formation/FileSource.cc \
../../../ImageProcessing/Formation/ImageIO.cc \
../../../ImageProcessing/Formation/JPEGIO.cc \
../../../ImageProcessing/Formation/PPMIO.cc \
../../../ImageProcessing/Formation/ImageBuffer.cc \
../../../ImageProcessing/Formation/Image.cc \
../../../ImageProcessing/Formation/RGBImage.cc \
../../../ImageProcessing/Formation/ImageConversion.cc \
../../../ImageProcessing/Formation/ImageSourceFactory.cc \
../../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
