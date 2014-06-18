TEMPLATE	= app
LANGUAGE	= C++

CONFIG	+= qt warn_on release

LIBS	+= -lqwt

INCLUDEPATH	+= /usr/include/qwt

HEADERS	+= OnlineRobotMonitorClient.h

SOURCES	+= main.cpp \
	OnlineRobotMonitorClient.cc

FORMS	= Monitor.ui

IMAGES	= images/filenew \
	images/fileopen \
	images/filesave \
	images/print \
	images/undo \
	images/redo \
	images/editcut \
	images/editcopy \
	images/editpaste \
	images/searchfind

unix {
  UI_DIR = .ui
  MOC_DIR = .moc
  OBJECTS_DIR = .obj
}

