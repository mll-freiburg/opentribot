TARGET = $(HOME)/.qt4/designer/libTacticsWidgetPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
TacticsWidgetPlugin.h \
../TacticsWidget.h \
../../../components/IDQObjects.h \

SOURCES = \
TacticsWidgetPlugin.cpp \
../TacticsWidget.cpp \
../../../components/IDQObjects.cpp \
../../States/RemoteBlackboard.cpp \
../../../../../Fundamental/Angle.cc \
../../../../../Fundamental/Vec.cc \
../../../../../Fundamental/Vec3D.cc \
../../../../../Fundamental/Time.cc \
../../../../../Fundamental/ConfigReader.cc \
../../../../../Fundamental/geometry.cc \
../../../../../Structures/TacticsBoard.cc \
../../../../../Structures/TacticsBoardDescription.cc \
../../../../../Structures/TeammateLocation.cc \
../../../../../Structures/ObstacleLocation.cc \
../../../../../Structures/GameState.cc \
../../../../../Structures/DriveVector.cc \
../../../../../Structures/RobotData.cc \
../../../../../Structures/FieldGeometry.cc \
../../../../../Structures/TribotsException.cc \
