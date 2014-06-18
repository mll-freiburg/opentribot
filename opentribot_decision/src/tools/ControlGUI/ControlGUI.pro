### TARGET: Dateiname der Zieldatei
TARGET = ../../../bin/ControlGUI

### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release

LIBS =
LIBPATH =
INCLUDEPATH =
FORMS = ControlGUI.ui
QTHEADERS = ../components/FieldOfPlay.h ../tribotsview/MonoLED.h DeactivateWidget.h


SOURCES = \
main.cpp \
DeactivateWidget.cpp \
../components/FieldOfPlay.cpp \
../components/CycleInfo.cpp \
../tribotsview/MonoLED.cpp \
../../Fundamental/ConfigReader.cc \
../../Fundamental/Angle.cc \
../../Fundamental/Vec.cc \
../../Fundamental/Time.cc \
../../Fundamental/stringconvert.cc \
../../Fundamental/geometry.cc \
../../Fundamental/Frame2D.cc \
../../Structures/TribotsException.cc \
../../Structures/DriveVector.cc \
../../Structures/RobotProperties.cc \
../../Structures/RobotData.cc \
../../Structures/ObstacleLocation.cc \
../../Structures/VisibleObject.cc \
../../Structures/GameState.cc \
../../Structures/FieldGeometry.cc \
../../Structures/TacticsBoard.cc \
../../Structures/MessageBoard.cc \
../../Structures/TeammateLocation.cc \
../../Structures/Journal.cc\
../../Communication/encoding.cc \
../../Communication/MultiPacketUDPCommunication.cc \
../../Communication/NonspecificTaggedUDPCommunication.cc \
../../Communication/PriorityUDPCommunication.cc \
../../Communication/TaggedUDPCommunication.cc \
../../Communication/TribotsUDPCommunication.cc \
../../Communication/UDPSocket.cc \
../../ImageProcessing/ObjectAnalysis/Regions.cc \
../../ImageProcessing/ObjectAnalysis/ChainCoding.cc \
