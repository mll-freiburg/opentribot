### TARGET: Dateiname der Zieldatei
TARGET = ../../../bin/LogCutter


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS =


### LIBS: programmspezifische Programmbibliotheken
LIBS = 


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = 


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = 


### QTHEADERS: Header, die QT-Makros enthalten
QTHEADERS = 


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES	= LogCutter.cc \
../../Fundamental/Angle.cc \
../../Fundamental/Vec.cc \
../../Fundamental/Time.cc \
../../Fundamental/binary_encoding.cc \
../../Fundamental/stringconvert.cc \
../../Structures/RobotLocationReadWriter.cc \
../../Structures/BallLocationReadWriter.cc \
../../Structures/ObstacleLocationReadWriter.cc \
../../Structures/VisibleObjectReadWriter.cc \
../../Structures/DriveVectorReadWriter.cc \
../../Structures/GameStateReadWriter.cc \
../../Structures/MessageBoardReadWriter.cc \
../../Structures/GameState.cc \
../../Structures/ObstacleLocation.cc \
../../Structures/VisibleObject.cc \
../../Structures/DriveVector.cc \
../../Structures/MessageBoard.cc
