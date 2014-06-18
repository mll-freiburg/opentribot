
SRCS     = ../../Fundamental/ConfigReader.cc
MAINSRCS = CanRobotTest.cc CanRobotKickerTest.cc
OBJS = $(SRCS:.cc=.o)
MAINOBJS = $(MAINSRCS:.cc=.o)
TRGT = $(MAINSRCS:.cc=)
DEPS = $(SRCS:.cc=.d) $(MAINSRCS:.cc=.d)

CC = g++
LC = g++
LIB_PATH = -L../../Libs/RobotCtr2/lib/  
LIBS = -lRobotCtr2 -lpcan -lpthread
INCLUDE_PATH = -I../../Libs/RobotCtr2/include
CFLAGS = -Wall -g 



# TARGETS:

all : $(TRGT)

% : %.o $(OBJS)
	@echo "linking $@"
	@$(LC) $(CFLAGS) $(LIB_PATH) -o $@ $@.o $(OBJS) $(LIBS)

%.d : %.cc
	@echo "making dependencies $@"
	@$(CC) -MM -MT "$@ $(@:.d=.o)" $(CFLAGS) $(INCLUDE_PATH) -MF $@ $<

%.o : %.cc
	@echo "compiling $<"
	@$(CC) $(CFLAGS) $(INCLUDE_PATH) -c -o $@ $<

clean:
	@echo "removing $(TRGT), .o-files, .d-files"
	@rm $(TRGT) $(OBJS) $(MAINOBJS) $(DEPS)


-include $(DEPS)
