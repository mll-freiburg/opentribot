
OBJ_DEST          := ./objs
INCL_DEST         := ./include

LIB_COMP          := ./src/components/lib/libcomponents.a
LIB_SERCOM        := ./src/SerCom/lib/libsercom.a
LIB_CTRBOARDS     := ./src/ctrBoards/lib/libctrboards.a
LIB_CTROMNIROBOT  := ./src/ctrOmniRobot/lib/libctromnirobot.a


librobotCtr.a: new $(LIB_COMP) $(LIB_SERCOM) $(LIB_CTRBOARDS) $(LIB_CTROMNIROBOT) $(INCL_DEST) $(OBJ_DEST)
	ar -x $(LIB_COMP)
	ar -x $(LIB_SERCOM)
	ar -x $(LIB_CTRBOARDS)
	ar -x $(LIB_CTROMNIROBOT)
	ar rc $@ *.o
	ranlib $@
	mv *.o $(OBJ_DEST)
	cp ./src/SerCom/include/*.h $(INCL_DEST)
	cp ./src/ctrBoards/include/*.h $(INCL_DEST)
	cp ./src/ctrOmniRobot/include/*.h $(INCL_DEST)

new:
	rm -rf $(LIB_COMP) $(LIB_SERCOM) $(LIB_CTRBOARDS) $(LIB_CTROMNIROBOT)
	rm -rf librobotCtr.a

$(OBJ_DEST):
	mkdir $(OBJ_DEST)

$(INCL_DEST):
	mkdir $(INCL_DEST)

$(LIB_COMP):
	cd ./src/components/src/; make; 

$(LIB_SERCOM):
	cd ./src/SerCom/src/; make;

$(LIB_CTRBOARDS):
	cd ./src/ctrBoards/src/; make;

$(LIB_CTROMNIROBOT):
	cd ./src/ctrOmniRobot/src/; make;

clean:
	rm -rf *.o *.d librobotCtr.a
	rm -rf $(OBJ_DIR)/*.o
	rm -rf $(INCL_DEST)/*.h
	cd ./src/ctrOmniRobot/src/; make veryclean;
	cd ./src/ctrBoards/src/; make veryclean;
	cd ./src/SerCom/src/; make veryclean;
	cd ./src/components/src/; make veryclean; 
