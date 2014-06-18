### TARGET: Dateiname der Zieldatei
TARGET = ../../../bin/ConnectionCloser


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS =


### LIBS: programmspezifische Programmbibliotheken
LIBS = 


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = 


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = \


### QTHEADERS: Header, die QT-Makros enthalten
HEADERS = \


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES	= \
ConnectionCloser.cpp \
../../Fundamental/Time.cc \
../../Fundamental/Vec.cc \
../../Fundamental/Angle.cc \
../../Fundamental/Vec3D.cc \
../../Fundamental/ConfigReader.cc \
../../Fundamental/geometry.cc \
../../Fundamental/stringconvert.cc \
../../Fundamental/Joystick.cc \
../../Fundamental/Frame2D.cc \
../../Structures/DriveVector.cc \
../../Structures/FieldGeometry.cc \
../../Structures/GameState.cc \
../../Structures/RobotProperties.cc \
../../Structures/RobotData.cc \
../../Structures/TacticsBoard.cc \
../../Structures/TacticsBoardDescription.cc \
../../Structures/TeammateLocation.cc \
../../Structures/TribotsException.cc \
../../Structures/ObstacleLocation.cc \
../../Structures/VisibleObject.cc \
../../Structures/MessageBoard.cc \
../../Structures/Journal.cc \
../../Structures/default_exception_handler.cc \
../../Communication/MultiPacketUDPCommunication.cc \
../../Communication/PriorityUDPCommunication.cc \
../../Communication/TaggedUDPCommunication.cc \
../../Communication/NonspecificTaggedUDPCommunication.cc \
../../Communication/TribotsUDPCommunication.cc \
../../Communication/UDPSocket.cc \
../../Communication/encoding.cc \
../../ImageProcessing/ObjectAnalysis/Regions.cc \
../../ImageProcessing/ObjectAnalysis/ChainCoding.cc \
