### TARGET: Dateiname der Zieldatei (default ist Projektdatei ohne .pro)
TARGET = ../../../bin/MultiCamMonitor


### CONFIG:
###           qt - QT-Anwendung
###          x11 - X11-Anwendung
###      warn_on - Warnungen des Compilers ausgeben (default)
###     warn_off - keine Compilerwarnungen ausgeben
###      release - optimiertes Uebersetzen
###        debug - Uebersetzen mit Debug-Information
###      profile - Erzeugen von Profiler-Information
###          app - Ausfuehrbares Programm erzeugen (default)
###   static_lib - Statische Bibliothek erzeugen
###   shared_lib - Dynamische Bibliothek erzeugen
CONFIG = qt warn_on debug


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS =


### LIBS: programmspezifische Programmbibliotheken
LIBS = raw1394 dc1394_control jpeg


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = ../../Libs/jpeg-6b


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = 


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = 


### QTHEADERS: Header, die QT-Makros enthalten (ausser von uic erstellte Header)
QTHEADERS = \
../components/ImageWidget.h \
SavingObject.h \

### SOURCES: Alle Quelldateien (.cpp, .cc, .c) (ausser von uic und moc erstellte Quellen)
SOURCES = \
main.cc \
../../tools/components/ImageWidget.cpp \
../../Fundamental/ConfigReader.cc \
../../Fundamental/Vec.cc \
../../Fundamental/geometry.cc \
../../Fundamental/Angle.cc \
../../Fundamental/Frame2D.cc \
../../Fundamental/stringconvert.cc \
../../Fundamental/Time.cc \
../../Structures/TribotsException.cc \
../../ImageProcessing/Formation/ImageProducer.cc \
../../ImageProcessing/Formation/MultiImageProducer.cc \
../../ImageProcessing/Formation/Image.cc \
../../ImageProcessing/Formation/RGBImage.cc \
../../ImageProcessing/Formation/YUVImage.cc \
../../ImageProcessing/Formation/Painter.cc \
../../ImageProcessing/Formation/ImageBuffer.cc \
../../ImageProcessing/Formation/IIDC.cc \
../../ImageProcessing/Formation/CamDriver.cc \
../../ImageProcessing/Formation/FileSource.cc \
../../ImageProcessing/Formation/ImageSourceFactory.cc \
../../ImageProcessing/Formation/UYVYImage.cc \
../../ImageProcessing/Formation/ImageIO.cc \
../../ImageProcessing/Formation/FileMonitor.cc \
../../ImageProcessing/Formation/JPEGIO.cc \
../../ImageProcessing/Formation/PPMIO.cc \
../../ImageProcessing/Formation/BufferedIO.cc \
../../ImageProcessing/Formation/ImageConversion.cc \
../../ImageProcessing/PixelAnalysis/RobotMask.cc \
../../ImageProcessing/PixelAnalysis/ColorClassifier.cc \
../../ImageProcessing/PixelAnalysis/YUVLookupTable.cc \
../../ImageProcessing/PixelAnalysis/Image2WorldMapping.cc \
../../ImageProcessing/PixelAnalysis/OmniCameraMapping.cc \
../../ImageProcessing/PixelAnalysis/DirectionalCameraMapping.cc \
../../ImageProcessing/Calibration/CameraOptics.cc \
../../Structures/Journal.cc \
../../Fundamental/geometry3D.cc \
../../Fundamental/Vec3D.cc \

