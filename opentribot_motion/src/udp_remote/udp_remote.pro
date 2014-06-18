TEMPLATE	= app
CONFIG		= console 
LIBS            += 
INCLUDEPATH     += ../components ../common
HEADERS		= \
../../../Fundamental/Joystick.h 
SOURCES		= \
../../../Fundamental/Joystick.cc \ 
../../../Structures/DriveVector.cc \
../../../Communication/UDPSocket.cc \
udp_remote.cpp


TARGET		= udp_remote

