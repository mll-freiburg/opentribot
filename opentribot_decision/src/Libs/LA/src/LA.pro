TARGET = ../lib/libLA.a

CONFIG = static_lib warn_on release

### HEADERS: list of header files that should be copied to directory HEADERDEST
HEADERS = \
eigenvalue.h \
indicator_array.h \
matrix.h \
matrix_compound_operations.h \
matrix_norms.h \
matrix_operations.h \
submatrix.h \

### HEADERDEST: directory to which HEADERS should be copied
HEADERDEST = ../include

### SOURCES: list of source files (.cpp, .cc, .c, .cxx, .C)
SOURCES = \
eigenvalue.cc \
matrix.cc \
matrix_compound_operations.cc \
matrix_norms.cc \
matrix_operations.cc \
submatrix.cc \
indicator_array.cc \
