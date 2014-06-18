
LIB_DEST        := ../lib
INCL_DEST       := ../include


CXX		:= g++
CXXINCLUDE	:= -I./

CXXLIBS		:= 

CXXFLAGS	:= -Wall

CSRCS		:= $(wildcard *.c)
CPPSRCS		:= $(wildcard *.cc)
SRCS		:= $(CSRCS) $(CPPSRCS)

COBJS		:= $(CSRCS:%.c=%.o)
CPPOBJS		:= $(CPPSRCS:%.cc=%.o)
OBJS		:= $(COBJS) $(CPPOBJS)

all: libcomponents.a $(LIB_DEST) $(INCL_DEST)
	cp libcomponents.a $(LIB_DEST)
	cp *.h $(INCL_DEST)

libcomponents.a: $(OBJS)
	ar rc $@ $(OBJS)
	ranlib $@

$(LIB_DEST):
	mkdir $(LIB_DEST)

$(INCL_DEST):
	mkdir $(INCL_DEST)


#erzeugt Abhaengigkeiten und schreibt die in Dateien mit Endung *.d
%.d: %.c
	@$(SHELL) -ec '$(CXX) -MM $(CXXFLAGS) $(CXXINCLUDE) $< \
                 | sed '\''s/\($*\)\.o[ :]*/\1.o $@: /g'\'' > $@'
	@(echo "   $@ generated")

#erzeugt Abhaengigkeiten und schreibt die in Dateien mit Endung *.d
%.d: %.cc
	@$(SHELL) -ec '$(CXX) -MM $(CXXFLAGS) $(CXXINCLUDE) $< \
                 | sed '\''s/\($*\)\.o[ :]*/\1.o $@: /g'\'' > $@'
	@(echo "   $@ generated")

#beachte alle Abhaengigkeitsdateien
include $(CPPSRCS:.cc=.d)
include $(CSRCS:.c=.d)

%.o: %.c	
	$(CXX) -c $(CXXFLAGS) $(CXXINCLUDE) $<

%.o: %.cc	
	$(CXX) -c $(CXXFLAGS) $(CXXINCLUDE) $<

clean:
	rm -f $(OBJS) *.o *.d *~ *.a

veryclean:
	rm -f $(OBJS) *.o *.d *~ *.a $(LIB_DEST)/*.a $(INCL_DEST)/*.h
