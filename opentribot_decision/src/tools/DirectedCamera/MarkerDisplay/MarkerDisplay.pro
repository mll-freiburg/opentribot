### TARGET: Dateiname der Zieldatei
TARGET = MarkerDisplay

### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = app qt warn_on debug

QTHEADERS = MarkerWidget.h

SOURCES = \
main.cpp \
MarkerWidget.cpp \
