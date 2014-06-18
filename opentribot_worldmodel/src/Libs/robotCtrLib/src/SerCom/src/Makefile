
LIB_DEST        := ../lib
INCL_DEST       := ../include

COMP_DIR        = ../../components

CXX		:= g++
CXXINCLUDE	:= -I./ -I$(COMP_DIR)/include

CXXLIBS		:= 

CXXFLAGS	:= -Wall

SRCS		:= $(wildcard *.cc)
OBJS		:= $(SRCS:%.cc=%.o)

all: libsercom.a $(LIB_DEST) $(INCL_DEST)
	cp libsercom.a $(LIB_DEST)
	cp sercom.h sercom_i.h $(INCL_DEST)

libsercom.a: $(OBJS)
	ar rc $@ $(OBJS)
	ranlib $@

$(LIB_DEST):
	mkdir $(LIB_DEST)

$(INCL_DEST):
	mkdir $(INCL_DEST)

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
	rm -f $(OBJS) *.o *.d *~ *.a

veryclean:
	rm -f $(OBJS) *.o *.d *~ *.a $(LIB_DEST)/*.a $(INCL_DEST)/*.h
