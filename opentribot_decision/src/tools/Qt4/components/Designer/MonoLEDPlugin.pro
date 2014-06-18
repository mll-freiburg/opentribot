TARGET = $(HOME)/.qt4/designer/libMonoLEDPlugin.so

CONFIG = plugin designer release

QTHEADERS = \
MonoLEDPlugin.h \
../MonoLED.h \

SOURCES = \
MonoLEDPlugin.cpp \
../MonoLED.cpp \
