
CXX             = g++
CXXFLAGS        = -g
CXXINCLUDE      += -I../include

LIBDEST		= ../lib


CCSRCS		= $(wildcard *.cc)
CCOBJS		= $(CCSRCS:.cc=.o)

CSRCS		= $(wildcard *.c)
COBJS		= $(CSRCS:.c=.o)

SRCS            = $(CCSRCS) $(CSRCS)
OBJS            = $(CCOBJS) $(COBJS)


TARGET          = libRobotCtr2.a

AR              = ar
ARFLAGS         = rc
RANLIB          = ranlib


default: $(TARGET) install


install: $(TARGET) $(LIBDEST)
	cp $(TARGET) $(LIBDEST)

$(LIBDEST):
	mkdir $(LIBDEST)

$(TARGET): $(OBJS)
	$(AR) $(ARFLAGS) $(TARGET) $(OBJS)
	$(RANLIB) $(TARGET)

#erzeugt Abhaengigkeiten und schreibt die in Dateien mit Endung *.d
%.d: %.c
	@$(SHELL) -ec '$(CXX) -MM $(CXXFLAGS) $(CXXINCLUDE) $< \
                 | sed '\''s/\($*\)\.o[ :]*/\1.o $@: /g'\'' > $@'
	@(echo "   $@ generated")

%.d: %.cc
	@$(SHELL) -ec '$(CXX) -MM $(CXXFLAGS) $(CXXINCLUDE) $< \
                 | sed '\''s/\($*\)\.o[ :]*/\1.o $@: /g'\'' > $@'
	@(echo "   $@ generated")

#beachte alle Abhaengigkeitsdateien
ifneq ($(strip $(CSRCS)),)
-include $(CSRCS:.c=.d)
endif
ifneq ($(strip $(CCSRCS)),)
-include $(CCSRCS:.cc=.d)
endif

%.o: %.c	
	$(CXX) -c $(CXXFLAGS) $(CXXINCLUDE) $<

%.o: %.cc	
	$(CXX) -c $(CXXFLAGS) $(CXXINCLUDE) $<

clean:
	rm -f *.d *.o semantic.cache $(TARGET) *~