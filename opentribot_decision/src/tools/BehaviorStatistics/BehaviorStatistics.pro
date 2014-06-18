### TARGET: Dateiname der Zieldatei (default ist Projektdatei ohne .pro)
TARGET = ../../../bin/BehaviorStatistics


### CONFIG:
###           qt - QT-Anwendung
###          x11 - X11-Anwendung
###      warn_on - Warnungen des Compilers ausgeben (default)
###     warn_off - keine Compilerwarnungen ausgeben
###      release - optimiertes Uebersetzen
###        debug - Uebersetzen mit Debug-Information
###      profile - Erzeugen von Profiler-Information
###          app - Ausfuehrbares Programm erzeugen (default)
###   static_lib - Statische Bibliothek erzeugen
###   shared_lib - Dynamische Bibliothek erzeugen
CONFIG = warn_on release


### DEFS: zusaetzliche Defines fuer den Compiler
DEFS =


### LIBS: programmspezifische Programmbibliotheken
LIBS = 


### LIBPATH: programmspezifische Bibliothekspfade
LIBPATH = 


### INCLUDEPATH: programmspezifische Include-Pfade
INCLUDEPATH = 


### FORMS: per designer erstellte Widgets (.ui-Dateien)
FORMS = 


### QTHEADERS: Header, die QT-Makros enthalten (ausser von uic erstellte Header)
QTHEADERS = 


### SOURCES: Alle Quelldateien (.cpp, .cc, .c) (ausser von uic und moc erstellte Quellen)
SOURCES = \
BehaviorStatistics.cc \
main.cc \
../../Fundamental/stringconvert.cc \
../../Fundamental/Time.cc \
../../Structures/GameState.cc \
../../Structures/GameStateReadWriter.cc \
../../Fundamental/binary_encoding.cc \
