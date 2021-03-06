
PHONY = clean clean.d clean.o clean.qt clean.h showsrcs showflags link help distclean all
PRO_GOALS = $(filter %.pro,$(MAKECMDGOALS))

ifeq (,$(filter help distclean all,$(MAKECMDGOALS)))
  # determine pro-file and include pro-file
  ifneq (,$(PRO_GOALS))
    ifeq (1,$(words $(PRO_GOALS)))
      PRO = $(word 1,$(PRO_GOALS))
    else
      $(error List of given project files is ambiguous)
    endif
  else
    -include .promake
    ifeq (, $(PRO))
      ALL_PROS = $(wildcard *.pro)
      NUM_PROS = $(words $(ALL_PROS))
      ifeq (1,$(NUM_PROS))
        PRO = $(ALL_PROS)
      else
        ifeq (0, $(NUM_PROS))
          $(error No project file found in working directory)
        else
          $(error Multiple project files found in working directory and none was chosen)
        endif
      endif
    endif
  endif
  ifeq (,$(PRO))
    $(error Did not find any project file)
  endif

  include $(PRO)
  CONFIG += $(TEMPLATE)    # compatibility with qmake
  QTFORMS += $(FORMS)   # compatibility with qmake

  ifneq (,$(strip $(filter dll plugin shared_lib sharedlib,$(CONFIG))))
    _PROTYPE = shared_lib
  else
    ifneq (,$(strip $(filter lib static_lib staticlib,$(CONFIG))))
      _PROTYPE = static_lib
    else
      _PROTYPE = app
    endif
  endif

  ifeq (,$(strip $(TARGET)))
    ifeq (app,$(_PROTYPE))
      TARGET = $(PRO:.pro=)
    else
      ifeq (static_lib,$(_PROTYPE))
        TARGET = $(addprefix lib,$(patsubst lib%,%,$(PRO:.pro=.a)))
      else
        TARGET = $(addprefix lib,$(patsubst lib%,%,$(PRO:.pro=.so)))
      endif
    endif
  endif

  ifneq (,$(DESTDIR))
    _TARGET := $(strip $(DESTDIR)/$(TARGET))   # compatibility with qmake
  else
    _TARGET = $(TARGET)
  endif
  _TARGETDIR = $(dir $(_TARGET))
  _FOUND_TARGETDIR = $(wildcard $(_TARGETDIR))
  ifneq ($(_TARGETDIR),$(_FOUND_TARGETDIR))
    $(warning creating target directory $(_TARGETDIR))
    $(shell mkdir $(_TARGETDIR))
  endif

  # generate list of source files, obj files, etc.
  ifneq (,$(filter qt,$(CONFIG)))
    ifeq (,$(strip $(QTHEADERS)))
      ifeq (,$(strip $(HEADERDEST)))
        QTHEADERS += $(HEADERS)    # compatibility with qmake
      else
        _HEADERS = $(HEADERS)
      endif
    else
      _HEADERS = $(HEADERS)
    endif
  else
    _HEADERS = $(HEADERS)
  endif
  ifneq (,$(strip $(HEADERDEST)))
    _HEADERDEST = $(strip $(HEADERDEST))
    _CPHEADERS = $(addprefix $(_HEADERDEST)/,$(notdir $(_HEADERS)))
    _FOUND_HEADERDEST = $(wildcard $(_HEADERDEST))
    ifneq ($(_HEADERDEST),$(_FOUND_HEADERDEST))
      $(warning creating include directory $(_HEADERDEST))
      $(shell mkdir $(_HEADERDEST))
    endif
  endif

  SUFFIXES = .c .cc .cpp .cxx .C
  UIHEADER = $(QTFORMS:.ui=.h)
  UISOURCES = $(QTFORMS:.ui=.cpp)
  MOCS = $(UIHEADER:.h=.moc.cpp) $(QTHEADERS:.h=.moc.cpp)
  OBJS = $(filter %.o,$(foreach suf,$(SUFFIXES),$(SOURCES:$(suf)=.o))) $(UISOURCES:.cpp=.o) $(MOCS:.cpp=.o)
  DEPS = $(filter %.d,$(foreach suf,$(SUFFIXES),$(SOURCES:$(suf)=.d))) $(UISOURCES:.cpp=.d) $(MOCS:.cpp=.d) $(QTFORMS:.ui=.ui.d)

  # some standard settings
  ifndef CXX
    CXX = g++
  endif
  LC = g++
  AR = ar rcs
  MOC = moc
  UIC = uic
  RM = rm -f
  CP = cp

  _LIBPATH = $(addprefix -L, $(LIBPATH)) $(filter -L%, $(LIBS))
  _INCLUDEPATH = $(addprefix -I, $(subst -I,,$(INCLUDEPATH)))
  _LIBS = $(addprefix -l, $(subst -l,,$(filter-out -L%, $(LIBS))))
  CXXFLAGS = $(addprefix -D, $(subst -D,,$(DEFS))) $(addprefix -D, $(subst -D,,$(DEFINES))) -export-dynamic

  # evaluate variable CONFIG 
  ifneq (,$(findstring qt,$(CONFIG)))
    ifndef QTDIR
      ifneq (,$(wildcard /usr/lib/qt3/include))
        QTDIR = /usr/lib/qt3
        $(warning environment variable QTDIR not set, trying to use /usr/lib/qt3) # Suse-Distribution
      else 
        ifneq (,$(wildcard /usr/share/qt3/include))
          QTDIR = /usr/share/qt3
          $(warning environment variable QTDIR not set, trying to use /usr/share/qt3) # Debian-Distribution
        else
          $(error environment variable QTDIR not set and no promising directory found)
        endif
      endif
    endif
    _LIBPATH += -L$(QTDIR)/lib
    _LIBS += -lqt-mt
    _INCLUDEPATH += -I$(QTDIR)/include
    CXXFLAGS += -DQT_NO_DEBUG -DQT_SHARED -DQT_THREAD_SUPPORT
  endif
  ifeq (,$(findstring warn_off,$(CONFIG)))
    CXXFLAGS += -Wall
  endif
  ifneq (,$(findstring debug,$(CONFIG)))
    CXXFLAGS += -g
  else
    CXXFLAGS += -DNDEBUG  
  endif
  ifneq (,$(findstring profile,$(CONFIG)))
    CXXFLAGS += -pg -fprofile-arcs
  endif
  ifneq (,$(findstring release,$(CONFIG)))
    CXXFLAGS += -O
  endif
  ifneq (,$(findstring thread,$(CONFIG)))
    _LIBS += -lpthread
  endif
  ifneq (,$(findstring x11,$(CONFIG)))
    ifndef X11DIR
       ifneq (,$(wildcard /usr/X11R6/include))
          X11DIR = /usr/X11R6
          $(warning environment variable X11DIR not set, trying to use /usr/X11R6)
       else
          $(error environment variable X11DIR not set)
       endif
    endif
    _LIBPATH += -L$(X11DIR)/lib
    _LIBS += -lXext -lX11 -lm
    _INCLUDEPATH += -I$(X11DIR)/include
  endif

  # binary, static library or shared library
  ifeq (app,$(_PROTYPE))
    LINKING_TEXT = "linking $(_TARGET)"
    LINKING = $(LC) $(CXXFLAGS) $(_LIBPATH) -o $(_TARGET) $(OBJS) $(_LIBS)
  else
    ifeq (static_lib,$(_PROTYPE))
      LINKING = $(AR) $(_TARGET) $(OBJS)
      LINKING_TEXT = "updating static library $(_TARGET)"
    else
      CXXFLAGS += -fPIC
      LINKING = $(LC) -shared $(CXXFLAGS) -o $(_TARGET) $(OBJS)
      LINKING_TEXT = "updating shared library $(_TARGET)"
    endif
  endif

else # end of: make-goal\not\in(help,distclean,all)

  _MAKEFILE = $(firstword $(MAKEFILE_LIST))
  _ALL_PRO = $(wildcard *.pro)

endif



# Goals:

.PHONY : $(PHONY) $(PRO)

$(_TARGET): $(UIHEADER) $(UISOURCES) $(MOCS) $(OBJS) $(DEPS) $(_CPHEADERS)
	@echo $(LINKING_TEXT)
	@$(LINKING)
	@echo "Success!"

$(PRO) :
	@echo "PRO = $(PRO)" > .promake
	@echo "working on project $(PRO)"

link: $(OBJS)
	@echo $(LINKING_TEXT)
	@$(LINKING)

%.ui.d :
	@echo "making dependencies $@"
	@echo "$(@:.ui.d=.h): $(@:.ui.d=.ui)" > $@

%.d : %.cc $(UIHEADER)
	@echo "making dependencies $@"
	@$(CXX) -MM -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(_INCLUDEPATH) -MF $@ $<

%.d : %.cpp $(UIHEADER)
	@echo "making dependencies $@"
	@$(CXX) -MM -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(_INCLUDEPATH) -MF $@ $<

%.d : %.cxx $(UIHEADER)
	@echo "making dependencies $@"
	@$(CXX) -MM -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(_INCLUDEPATH) -MF $@ $<

%.d : %.C $(UIHEADER)
	@echo "making dependencies $@"
	@$(CXX) -MM -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(_INCLUDEPATH) -MF $@ $<

%.d : %.c $(UIHEADER)
	@echo "making dependencies $@"
	@$(CXX) -MM -MT "$@ $(@:.d=.o)" $(CXXFLAGS) $(_INCLUDEPATH) -MF $@ $<

$(_CPHEADERS) : $(_HEADERS)
	@echo "copying header files $(_HEADERS)"
	@$(CP) $(_HEADERS) $(_HEADERDEST)

%.h : %.ui
	@echo "$(UIC)-ing $@"
	@cd $(<D); $(UIC) $(<F) -o $(@F)

%.cpp : %.ui %.ui.h
	@echo "$(UIC)-ing $@"
	@cd $(<D); $(UIC) $(<F) -i $(*F).h -o $(@F)

%.moc.cpp : %.h
	@echo "$(MOC)-ing $@"
	@cd $(<D); $(MOC) $(<F) -o $(@F)

%.o : %.cc
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(_INCLUDEPATH) -c -o $@ $<

%.o : %.c
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(_INCLUDEPATH) -c -o $@ $<

%.o : %.cpp
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(_INCLUDEPATH) -c -o $@ $<

%.o : %.cxx
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(_INCLUDEPATH) -c -o $@ $<

%.o : %.C
	@echo "compiling $<"
	@$(CXX) $(CXXFLAGS) $(_INCLUDEPATH) -c -o $@ $<

clean: clean.d clean.o clean.qt clean.h
	@echo "removing $(_TARGET)"
	@$(RM) $(_TARGET)

clean.d:
	@echo "removing .d-files"
	@$(RM) $(DEPS)

clean.o:
	@echo "removing .o-files"
	@$(RM) $(OBJS)

clean.qt:
	@echo "removing uic- and moc-generated files"
	@$(RM) $(MOCS) $(UIHEADER) $(UISOURCES)

clean.h:
	@echo "removing copied .h-files"
	@$(RM) $(_CPHEADERS)

distclean: 
	@echo "cleaning all projects in current directory: $(_ALL_PRO)"
	@+$(foreach PRO1, $(_ALL_PRO), \make -s -f $(_MAKEFILE) $(PRO1); \make -s -f $(_MAKEFILE) clean;)
	@$(RM) .promake
	@echo "removing file .promake"

all:
	@echo "making all projects in current directory: $(_ALL_PRO)"
	@+$(foreach PRO1, $(_ALL_PRO), \make -s -f $(_MAKEFILE) $(PRO1); \make -s -f $(_MAKEFILE) ;)

showsrcs:
	@echo $(SOURCES) $(UISOURCES) $(MOCS)

showflags:
	@echo $(CXXFLAGS) $(_INCLUDEPATH) $(_LIBPATH) $(_LIBS)

help:
	@echo "Makefile for middle-sized C/C++-projects:"
	@echo "  - supports binary generation, static and shared libraries."
	@echo "  - supports X11- and QT-applications."
	@echo "  - uses project files similar to QT-projectfiles."
	@echo "  - exemplary project file: default.pro"
	@echo "  - uses the only project file in the working directory, or the"
	@echo "    project file that has been specified with \"make xxx.pro\""
	@echo 
	@echo "Available Make-goals:"
	@echo "   (empty): generate binary/library"
	@echo "       all: make all projects in current directory"
	@echo "      link: only linking, do not consult dependencies"
	@echo "     clean: remove all generated files"
	@echo "   clean.d: remove .d-files describing dependencies"
	@echo "   clean.h : remove copied .h-files"
	@echo "   clean.o: remove .o-files"
	@echo "  clean.qt: remove files generated by uic and moc"
	@echo " distclean: remove all projects in current directory"
	@echo "  showsrcs: show list of C/C++-sources (.c, .cc, .cpp, .cxx, .C)"
	@echo " showflags: show list of compiler/linker flags"
	@echo 
	@echo "environment variables:"
	@echo " QTDIR defines path to QT headers and libraries"
	@echo "X11DIR defines path to X11 headers and libraries"
	@echo "   CXX defines C++ compiler (g++ if variable is undefined)"



# dependencies:

ifeq (,$(filter $(PHONY) $(PRO_GOALS), $(MAKECMDGOALS)))
  -include $(DEPS)
endif
