TARGET = ../../../bin/marker_editor
CONFIG = qt warn_on debug
LIBS = LA
LIBPATH = ../../Libs/LA/lib
INCLUDEPATH = ../../Libs/LA/include

QTHEADERS = \
qt_gui/helpwindow.h \
qt_gui/paper.h \
qt_gui/selwindow.h \
qt_gui/topwindow.h \

SOURCES = \
../CalibrationTool/DistMarkerBuilder.cpp \
qt_gui/helpwindow.cc \
qt_gui/main.cc \
qt_gui/paper.cc \
qt_gui/selwindow.cc \
qt_gui/topwindow.cc \
../../Fundamental/Vec.cc \
../../Fundamental/Frame2D.cc \
../../Fundamental/Angle.cc \
../../Fundamental/geometry.cc \
../../Fundamental/ConfigReader.cc \
../../Structures/TribotsException.cc \

