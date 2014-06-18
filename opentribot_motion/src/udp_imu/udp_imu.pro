TEMPLATE	= app
CONFIG		= console 
DEFINES		+= USE_USB_IMU
LIBS            += -lusb 
INCLUDEPATH     += ../components ../common usbimu
SOURCES		= \
usbimu/imu.cc \
UDPSocket.cc \
udp_imu.cpp


TARGET		= udp_imu

