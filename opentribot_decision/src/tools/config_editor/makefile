# i.d.R. sollte dieses Makefile nur verwendet werden, wenn genau 
# ein *.pro-File im aktuellen Verzeichnis ist. Wenn nicht, wird das 
# alphabetisch erste genommen. Verarbeitet QT- und X11-Anwendungen

PHONY = clean clean.d clean.o clean.qt showsrcs link help

ifeq (,$(findstring help,$(MAKECMDGOALS)))
  PRO = $(word 1,$(wildcard *.pro))
  ifeq (,$(PRO))
    $(error "Keine .pro-Datei gefunden")
  endif

  TARGET=$(PRO:.pro=) 
  include $(PRO)

  SUFFIXES = .c .cc .cpp
  UIHEADER = $(FORMS:.ui=.h)
  UISOURCES = $(FORMS:.ui=.cpp)
  MOCS = $(UIHEADER:.h=.moc.cpp) $(QTHEADERS:.h=.moc.cpp)
  OBJS = $(filter %.o,$(foreach suf,$(SUFFIXES),$(SOURCES:$(suf)=.o))) $(UISOURCES:.cpp=.o) $(MOCS:.cpp=.o)
  DEPS = $(filter %.d,$(foreach suf,$(SUFFIXES),$(SOURCES:$(suf)=.d))) $(UISOURCES:.cpp=.d) $(MOCS:.cpp=.d) $(FORMS:.ui=.ui.d)

  CC = g++
  LC = g++
  AR = ar
  MOC = moc
  UIC = uic
  
  LIB_PATH = $(addprefix -L, $(LIBPATH))
  INCLUDE_PATH = $(addprefix -I, $(INCLUDEPATH))
  LIBRARIES = $(addprefix -l, $(LIBS))
  CFLAGS = $(addprefix -D, $(DEFS)) -export-dynamic


  # CONFIG auswerten
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
    LIB_PATH += -L$(QTDIR)/lib
    LIBRARIES += -lqt-mt
    INCLUDE_PATH += -I$(QTDIR)/include
    CFLAGS += -DQT_NO_DEBUG -DQT_SHARED -DQT_THREAD_SUPPORT
  endif
  ifeq (,$(findstring warn_off,$(CONFIG)))
    CFLAGS += -Wall
  endif
  ifneq (,$(findstring debug,$(CONFIG)))
    CFLAGS += -g
  endif
  ifneq (,$(findstring profile,$(CONFIG)))
    CFLAGS += -pg -fprofile-arcs
  endif
  ifneq (,$(findstring release,$(CONFIG)))
    CFLAGS += -O
  endif
  ifneq (,$(findstring x11,$(CONFIG)))
    ifndef X11DIR
      $(error "Umgebungsvariable X11DIR undefiniert")
    endif
    LIB_PATH += -L$(X11DIR)/lib
    LIBRARIES += -lXext -lX11 -lm
    INCLUDE_PATH += -I$(X11DIR)/include
  endif

  # Anwendung, statische oder dynamische Bibliothek
  LINKING_TEXT = "linking $(TARGET)"
  LINKING = $(LC) $(CFLAGS) $(LIB_PATH) -o $(TARGET) $(OBJS) $(LIBRARIES)
  ifneq (,$(findstring static_lib,$(CONFIG)))
    LINKING = $(AR) r $(TARGET) $(OBJS)
    LINKING_TEXT = "updating static library $(TARGET)"
  endif
  ifneq (,$(findstring shared_lib,$(CONFIG)))
    CFLAGS += -fPIC
    LINKING = $(LC) -shared $(CFLAGS) -o $(TARGET) $(OBJS)
    LINKING_TEXT = "updating shared library $(TARGET)"
  endif


endif    # Ende von make-Ziel!=help?




# Ziele:

$(TARGET): $(UIHEADER) $(UISOURCES) $(MOCS) $(OBJS) $(DEPS)
	@echo $(LINKING_TEXT)
	@$(LINKING)
	@echo "Success!"

link: $(OBJS)
	@echo $(LINKING_TEXT)
	@$(LINKING)

%.ui.d :
	@echo "making dependencies $@"
	@echo "$(@:.ui.d=.h): $(@:.ui.d=.ui)" > $@

%.d : %.cc $(UIHEADER)
	@echo "making dependencies $@"
	@$(CC) -MM -MT "$@ $(@:.d=.o)" $(CFLAGS) $(INCLUDE_PATH) -MF $@ $<

%.d : %.cpp $(UIHEADER)
	@echo "making dependencies $@"
	@$(CC) -MM -MT "$@ $(@:.d=.o)" $(CFLAGS) $(INCLUDE_PATH) -MF $@ $<

%.d : %.c $(UIHEADER)
	@echo "making dependencies $@"
	@$(CC) -MM -MT "$@ $(@:.d=.o)" $(CFLAGS) $(INCLUDE_PATH) -MF $@ $<

%.h : %.ui
	@echo "$(UIC)-ing $@"
	@$(shell cd $(<D); $(UIC) $(<F) -o $(@F))

%.cpp : %.ui %.ui.h
	@echo "$(UIC)-ing $@"
	@$(shell cd $(<D); $(UIC) $(<F) -i $(*F).h -o $(@F))

%.moc.cpp : %.h
	@echo "$(MOC)-ing $@"
	@$(shell cd $(<D); $(MOC) $(<F) -o $(@F))

%.o : %.cc
	@echo "compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDE_PATH) -c -o $@ $<

%.o : %.c
	@echo "compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDE_PATH) -c -o $@ $<

%.o : %.cpp
	@echo "compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDE_PATH) -c -o $@ $<

clean:
	@echo "removing $(TARGET), .o-files, .d-files, .moc.cpp-files, uic-h-files, uic-cpp-files"
	@rm $(OBJS) $(TARGET) $(DEPS) $(MOCS) $(UISOURCES) $(UIHEADER)

clean.d:
	@echo "removing .d-files"
	@rm $(DEPS)

clean.o:
	@echo "removing .o-files"
	@rm $(OBJS)

clean.qt:
	@echo "removing uic- and moc-generated files"
	@rm $(MOCS) $(UIHEADER) $(UISOURCES)

showsrcs:
	@echo $(SOURCES) $(UISOURCES) $(MOCS)

help:
	@echo "Make fuer mittelgrosse Projekte:"
	@echo "  Unterstuetzung fuer Anwendungen und Bibliotheken."
	@echo "  Unterstuetzung fuer X11- und QT-Anwendungen."
	@echo "  Liest spezifische Informationen aus Projektdateien."
	@echo "  Beispiel fuer Projektdatei siehe: demo.pro"
	@echo "  Waehlt jeweils die erste der .pro-Dateien im"
	@echo "  aktuellen Arbeitsverzeichnis."
	@echo 
	@echo "Ziele:"
	@echo "   (leer): Anwendung/Bibliothek erzeugen"
	@echo "     link: nur linken/Bibliothek erzeugen, .o-Dateien nicht neu erzeugen"
	@echo "    clean: Alle erzeugten Dateien loeschen"
	@echo "  clean.d: Die .d-Dateien (Abhaengigkeiten) loeschen"
	@echo "  clean.o: Die .o-Dateien loeschen"
	@echo " clean.qt: Die von uic und moc erzeugten Dateien loeschen"
	@echo " showsrcs: Liste der Quelldateien (.c-, .cc-, .cpp-Dateien)"


 
# Abhaengigkeiten:  
   
ifeq (,$(findstring $(MAKECMDGOALS),$(PHONY)))
  -include $(DEPS)
endif
