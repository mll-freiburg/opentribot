######################################################################
# Automatically generated by qmake (1.07a) Tue Apr 14 15:21:52 2009
######################################################################

TEMPLATE = app
INCLUDEPATH += . ../udp_imu/usbimu/
LIBS += -lusb

DEFINES += USE_USB_KICKER_IMU


# Input
HEADERS += kick.h
HEADERS += StopWatch.h
SOURCES += kick.cc
SOURCES += ../udp_imu/usbimu/imu.cc
