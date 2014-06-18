### TARGET: Dateiname der Zieldatei
TARGET = gyro_test


### CONFIG:
###       qt - QT-Anwendung
###  warn_on - Warnungen des Compilers ausgeben
###  release - optimiertes Uebersetzen
###    debug - Uebersetzen mit Debug-Information
CONFIG = warn_on release thread


### SOURCES: Alle Quelldateien (.cpp, .cc, .c)
SOURCES	= \
gyro_test.cc \
../../Fundamental/Time.cc \
../../Robot/CompassGrabbingThread.cc \
../../Fundamental/POSIXThread.cc \
