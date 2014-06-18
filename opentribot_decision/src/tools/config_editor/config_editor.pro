### TARGET: Dateiname der Zieldatei
TARGET = ../../../bin/config_editor


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = qt warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS =


### LIBS: programmspezifische Programmbibliotheken
LIBS = 


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = 


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = mainWidget.ui


### QTHEADERS: Header, die QT-Makros enthalten
QTHEADERS =


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES	= \
main.cpp \
TribotsSyntax.cpp \
../../Fundamental/ConfigReader.cc
