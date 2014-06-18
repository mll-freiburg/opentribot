
COMP_DIR        := ../../components
SER_COM_DIR     := ../

CXX		:= g++
CXXINCLUDE	:= -I$(SER_COM_DIR)/include -I$(COMP_DIR)/include

CXXLIBS		:= -L$(SER_COM_DIR)/lib -lsercom -L$(COMP_DIR)/lib -lcomponents

CSRCS		:= $(wildcard *.c)
CPPSRCS		:= $(wildcard *.cpp)
SRCS		:= $(CSRCS) $(CPPSRCS)

COBJS		:= $(CSRCS:%.c=%.o)
CPPOBJS		:= $(CPPSRCS:%.cpp=%.o)
OBJS		:= $(COBJS) $(CPPOBJS)

all: host

host:   $(OBJS)
	$(CXX) $(CXXINCLUDE) -o $@ host.o $(CXXLIBS)

#erzeugt Abhaengigkeiten und schreibt die in Dateien mit Endung *.d
%.d: %.c
	@$(SHELL) -ec '$(CXX) -MM $(CXXFLAGS) $(CXXINCLUDE) $< \
                 | sed '\''s/\($*\)\.o[ :]*/\1.o $@: /g'\'' > $@'
	@(echo "   $@ generated")

#erzeugt Abhaengigkeiten und schreibt die in Dateien mit Endung *.d
%.d: %.cpp
	@$(SHELL) -ec '$(CXX) -MM $(CXXFLAGS) $(CXXINCLUDE) $< \
                 | sed '\''s/\($*\)\.o[ :]*/\1.o $@: /g'\'' > $@'
	@(echo "   $@ generated")

#beachte alle Abhaengigkeitsdateien
include $(CPPSRCS:.cpp=.d)
include $(CSRCS:.c=.d)

%.o: %.c	
	$(CXX) -c $(CXXFLAGS) $(CXXINCLUDE) $<

%.o: %.cpp	
	$(CXX) -c $(CXXFLAGS) $(CXXINCLUDE) $<

clean:
	rm -f $(OBJS) *.o *.d *.a host client

veryclean:
	rm -f $(OBJS) *.o *.d *~ *.a host  *.errlog