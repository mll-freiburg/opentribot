
SRCS = ../../Communication/UDPSocket.cc ../../Communication/MultiPacketUDPCommunication.cc ../../Communication/PriorityUDPCommunication.cc ../../Communication/TaggedUDPCommunication.cc ../../Fundamental/Time.cc
MAINSRCS = test_UDPSocket_client.cc test_UDPSocket_server.cc test_MultiPacketUDPCommunication_client.cc test_MultiPacketUDPCommunication_server.cc test_PriorityUDPCommunication_client.cc test_PriorityUDPCommunication_server.cc test_TaggedUDPCommunication_client.cc test_TaggedUDPCommunication_server.cc
OBJS = $(SRCS:.cc=.o)
MAINOBJS = $(MAINSRCS:.cc=.o)
TRGT = $(MAINSRCS:.cc=)
DEPS = $(SRCS:.cc=.d) $(MAINSRCS:.cc=.d)

CC = g++
LC = g++
LIB_PATH = 
LIBS = 
INCLUDE_PATH = 
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
