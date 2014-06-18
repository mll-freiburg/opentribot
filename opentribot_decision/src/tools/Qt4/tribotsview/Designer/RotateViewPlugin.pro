TARGET = $(HOME)/.qt4/designer/libRotateViewPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
RotateViewPlugin.h \
../RotateView.h \
../../components/FieldOfPlay.h \

SOURCES = \
RotateViewPlugin.cpp \
../RotateView.cpp \
../../components/FieldOfPlay.cpp \
../../../../Fundamental/Vec.cc \
../../../../Fundamental/Time.cc \
../../../../Structures/ObstacleLocation.cc \
../../../../Structures/VisibleObject.cc \
../../../../Structures/GameState.cc \
../../../../Structures/DriveVector.cc \
../../../../Fundamental/Frame2D.cc \
../../../../Fundamental/Vec3D.cc \
../../../../Fundamental/Angle.cc \
../../../../Structures/TribotsException.cc \
../../../../Fundamental/geometry.cc \
../../../../Structures/FieldGeometry.cc \
../../../../Fundamental/ConfigReader.cc \
../../components/CycleInfo.cpp \
