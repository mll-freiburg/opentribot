LIBDEST         = ../lib
INCDEST         = ../include

COMP_DIR        = ../../components
SERCOM_DIR      = ../../SerCom

CXX		= g++
CXXINCLUDE	= -I$(SERCOM_DIR)/include -I./ -I$(COMP_DIR)/include

CXXLIBS		= 

CXXFLAGS	= -Wall

SRCS		= Tmc200.cc SingleDrive.cc 
OBJS		= $(SRCS:%.cc=%.o)
HEADERS         = Tmc200.h SingleDrive.h

all: libctrboards.a $(LIBDEST) $(INCDEST)
	cp libctrboards.a $(LIBDEST)
	cp $(HEADERS) $(INCDEST)

libctrboards.a: $(OBJS)
	ar rc $@ $(OBJS)	
	ranlib $@

$(LIBDEST):
	mkdir $(LIBDEST)

$(INCDEST):
	mkdir $(INCDEST)

#erzeugt Abhaengigkeiten und schreibt die in Dateien mit Endung *.d
%.d: %.cc
	@$(SHELL) -ec '$(CXX) -MM $(CXXFLAGS) $(CXXINCLUDE) $< \
                 | sed '\''s/\($*\)\.o[ :]*/\1.o $@: /g'\'' > $@'
	@(echo "   $@ generated")

#beachte alle Abhaengigkeitsdateien
include $(SRCS:.cc=.d)

%.o: %.cc	
	$(CXX) -c $(CXXFLAGS) $(CXXINCLUDE) $<

clean:
	rm -f $(OBJS) *.d

veryclean:
	rm -f $(OBJS) libctrboards.a $(LIBDEST)/libctrboards.a $(INCDEST)/*.h *~ *.d
