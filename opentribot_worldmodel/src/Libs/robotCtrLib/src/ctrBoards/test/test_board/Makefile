.SUFFIXES: .C $(SUFFIXES)

COMP_DIR        = ../../../components
SERCOM_DIR      = ../../../SerCom
CTRBOARD_DIR    = ../../

CXX		= g++
CXXINCLUDE	= -I$(CTRBOARD_DIR)/include \
		  -I$(SERCOM_DIR)/include   \
                  -I$(COMP_DIR)/include

CXXLIBS		= -L $(CTRBOARD_DIR)/lib -lctrboards \
		  -L $(COMP_DIR)/lib -lcomponents \
                  -L $(SERCOM_DIR)/lib -lsercom

CXXFLAGS	= -Wall

SRCS		= test.cc
OBJS		= $(SRCS:%.cc=%.o)


test: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(CXXLIBS) 

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
	rm -f $(OBJS) 