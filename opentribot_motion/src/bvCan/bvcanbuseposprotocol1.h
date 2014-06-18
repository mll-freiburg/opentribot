#ifndef BV_CAN_BUS_PROTOCOL1
#define BV_CAN_BUS_PROTOCOL1

#include "bvdebug.h"
#include "bvmessage.h"
#include "bvcanopenmessage.h"
#include "bvcanopenobject.h"
#include "bvcanopenmessageset.h"

class BVCanBusEposProtocol1{
public:
  enum enumCommunicationState{ good=0,waitForEposResponce,unknownCanID};

  enum enumEposState{ beforeStartState =0,
                      getStatus,                  getStatusSent,
                      switchOnDisabled ,          shutDownSent,
                      readyToSwitchOn,            switchOnSent, 
                      switchedOn,                 enabledOperationSent,     
                      operationEnabled,    
                      sentTargetVelocity,         recievedTargetVelocity,
                      sentCwAfterTargetVelocity , recievedCwAfterTargetVelocity ,
                      sentVelSensorActValue,      recievedVelSensorActValue ,
                      sentCurrentActValue,        recievedCurrentActValue,
                      sentPositionActValue,       idleState,
                      fault,
                      sentFaultReset , recievedFaultReset
                      };

  enum enumCurrentMsgIndex{ 
                            idleMessage=0,
                            initCW0,
                            initCW1,
                            initCW2,
                            initSW0,
                            targetVelocity,
                            CWAfterTargetVelocity,
                            velSensorActValue,
                            currentActValue ,
                            positionActValue,
                            initCWReset};

public:
  BVCanBusEposProtocol1(int aNodeID0 , int aNodeID1 , int aNodeID2 , BVCanOpenMessageSet *anObjectsSet);
public:
  void sentMessage( BVCanOpenObject* obj );
  void recievedMessage( BVCanOpenObject* obj);
  bool getNextObjToSend(int index , BVCanOpenObject** anObj );
  void setToState(int index , enum enumEposState aEposState, enum enumCurrentMsgIndex anextMsgIndex);
  enum enumEposState getEposState(int index , string* aname   );
  bool isSent( int index);
public:
  int getNodeID( int index );
  
private:
  int nodeID[3];
private:
  void updateEposState( int index , BVCanOpenObject* obj , BVCanOpenObject::directionEnum adirection);
private:
  // ................................................................
  // ................................................................
  // holds the current state of the 3 state machines for the 3 EPOS
  enum enumEposState eposState[3];
  // ................................................................
  // ................................................................
private:
  BVCanOpenMessageSet *objectsSet;
  BVCanOpenObject* objs[3*11];
  int numberOfObjs; 
  int nextObjToSendIndex[3];

  string stateName[23];

 
};

#endif
