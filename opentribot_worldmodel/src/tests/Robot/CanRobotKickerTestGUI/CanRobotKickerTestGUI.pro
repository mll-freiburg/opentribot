TEMPLATE	= app
LANGUAGE	= C++

CONFIG	+= qt warn_on release

LIBS	+= ../../../Libs/RobotCtr2/lib/libRobotCtr2.a -lpcan

INCLUDEPATH	+= ../../../Libs/RobotCtr2/include

SOURCES	+= main.cpp

FORMS	= mainform.ui

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

