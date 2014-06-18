TARGET = $(HOME)/.qt4/designer/libFieldOfPlayPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
FieldOfPlayPlugin.h \
../FieldOfPlay.h \

SOURCES = \
FieldOfPlayPlugin.cpp \
../FieldOfPlay.cpp \
../CycleInfo.cpp \
../../../../Structures/ObstacleLocation.cc \
../../../../Structures/VisibleObject.cc \
../../../../Structures/GameState.cc \
../../../../Structures/DriveVector.cc \
../../../../Fundamental/geometry.cc \
../../../../Fundamental/Frame2D.cc \
../../../../Fundamental/Vec3D.cc \
../../../../Fundamental/Vec.cc \
../../../../Fundamental/Angle.cc \
../../../../Fundamental/Time.cc \
../../../../Structures/FieldGeometry.cc \
../../../../Fundamental/ConfigReader.cc \
../../../../Structures/TribotsException.cc \
../../../../Fundamental/stringconvert.cc
