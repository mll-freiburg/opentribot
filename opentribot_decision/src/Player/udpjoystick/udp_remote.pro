TEMPLATE	= app
CONFIG		= console 
LIBS            += 
INCLUDEPATH     += ../components ../common
HEADERS		= \
../../Fundamental/Joystick.h 
SOURCES		= \
../../Fundamental/Joystick.cc \ 
../../Communication/UDPSocket.cc \
udp_remote.cc


TARGET		= udp_remote

