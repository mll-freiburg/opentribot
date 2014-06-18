
TARGET = odeserver

#DEFINES += NO_X

CONFIG = warn_on release

LIBPATH = lib /usr/X11R6/lib

LIBS = -ldrawstuff -lode -lX11 -lGL -lGLU -lpthread
#LIBS = -ldrawstuff -lode -lpthread -framework GLUT -framework OpenGL -framework Cocoa

INCLUDEPATH = .

SOURCES = \
Simulator/Communication.cpp \
Simulator/World.cpp \
Simulator/dynamics.cpp \
Simulator/external.cpp \
Simulator/helpers.cpp \
Simulator/main.cpp \
Communication/SimulatorUDPCommunication.cpp \
Communication/NonspecificTaggedUDPCommunication.cc \
Communication/TaggedUDPCommunication.cc \
Communication/PriorityUDPCommunication.cc \
Communication/MultiPacketUDPCommunication.cc \
Communication/UDPSocket.cc \
Fundamental/Time.cc \
