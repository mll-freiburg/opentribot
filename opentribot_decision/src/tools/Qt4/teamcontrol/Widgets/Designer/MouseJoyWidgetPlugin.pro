TARGET = $(HOME)/.qt4/designer/libMouseJoyWidgetPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
MouseJoyWidgetPlugin.h \
../MouseJoyWidget.h \

SOURCES = \
MouseJoyWidgetPlugin.cpp \
../MouseJoyWidget.cpp \
../../States/RemoteBlackboard.cpp \
../../../../../Fundamental/Angle.cc \
../../../../../Fundamental/Vec.cc \
../../../../../Fundamental/Vec3D.cc \
../../../../../Fundamental/Time.cc \
../../../../../Fundamental/ConfigReader.cc \
../../../../../Structures/TacticsBoard.cc \
../../../../../Structures/TacticsBoardDescription.cc \
../../../../../Structures/TeammateLocation.cc \
../../../../../Structures/ObstacleLocation.cc \
../../../../../Structures/GameState.cc \
../../../../../Structures/DriveVector.cc \
../../../../../Structures/RobotData.cc \
../../../../../Structures/FieldGeometry.cc \
../../../../../Structures/MessageBoard.cc \
../../../../../Structures/VisibleObject.cc \
../../../../../Structures/TribotsException.cc \
../../../../../Fundamental/geometry.cc \
../../../../../Fundamental/Frame2D.cc \
../../../../../Fundamental/stringconvert.cc \
