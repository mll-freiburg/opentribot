
CXX             = g++
CXXFLAGS        = -g

RCLPATH	        = ../../

CXXINCLUDE      += -I$(RCLPATH)/include
CXXLIBS		= -lpcan -pthread -L $(RCLPATH)/lib -lRobotCtr2


CCSRCS		= $(wildcard *.cc)
CCOBJS		= $(CCSRCS:.cc=.o)

CSRCS		= $(wildcard *.c)
COBJS		= $(CSRCS:.c=.o)

SRCS            = $(CCSRCS) $(CSRCS)
OBJS            = $(CCOBJS) $(COBJS)


TARGET          = cmdline

default: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o$(TARGET) $(CXXINCLUDE) $(OBJS) $(CXXLIBS)

all:
	$(CXX) -o$(TARGET) $(CXXINCLUDE) $(OBJS) $(CXXLIBS)

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