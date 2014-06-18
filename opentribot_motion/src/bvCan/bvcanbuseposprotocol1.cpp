#include "bvcanbuseposprotocol1.h"
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// -- ctor ----------------------------------------------------------------
// @param1, aNodeID0 , for example 0x02
// @param2, aNodeID1 , for example 0x04
// @param3, aNodeID2 , for example 0x08
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
BVCanBusEposProtocol1::BVCanBusEposProtocol1( int aNodeID0 , int aNodeID1 , int aNodeID2 , BVCanOpenMessageSet *anObjectsSet){

  nodeID[0]=aNodeID0;
  nodeID[1]=aNodeID1;
  nodeID[2]=aNodeID2;
  objectsSet=anObjectsSet;

  numberOfObjs = 11;
  
  for(int index =0;index<3;index++){

    *(objs+index*numberOfObjs+idleMessage)            = NULL;
    *(objs+index*numberOfObjs+initCW0)                = new BVCanOpenMessageControlWord(       * ((BVCanOpenMessageControlWord*)(objectsSet->objs[ID_BVCanOpenMessageControlWord])));
    *(objs+index*numberOfObjs+initCW1)                = new BVCanOpenMessageControlWord(       * ((BVCanOpenMessageControlWord*)(objectsSet->objs[ID_BVCanOpenMessageControlWord])));
    *(objs+index*numberOfObjs+initCW2)                = new BVCanOpenMessageControlWord(       * ((BVCanOpenMessageControlWord*)(objectsSet->objs[ID_BVCanOpenMessageControlWord])));

    *(objs+index*numberOfObjs+initSW0)                = new BVCanOpenMessageStatusWord(        * ((BVCanOpenMessageStatusWord*)(objectsSet->objs[ID_BVCanOpenMessageStatusWord])));

    *(objs+index*numberOfObjs+targetVelocity)         = new BVCanOpenMessageTargetVelocity(    * ((BVCanOpenMessageTargetVelocity*)(objectsSet->objs[ID_BVCanOpenMessageTargetVelocity])) );
    *(objs+index*numberOfObjs+CWAfterTargetVelocity)  = new BVCanOpenMessageControlWord(       * ((BVCanOpenMessageControlWord*)(objectsSet->objs[ID_BVCanOpenMessageControlWord])));
    *(objs+index*numberOfObjs+velSensorActValue)      = new BVCanOpenMessageVelSensorActValue( * ((BVCanOpenMessageVelSensorActValue*)(objectsSet->objs[ID_BVCanOpenMessageVelSensorActValue])));
    *(objs+index*numberOfObjs+currentActValue)        = new BVCanOpenMessageCurrentActValue(   * ((BVCanOpenMessageCurrentActValue*)(objectsSet->objs[ID_BVCanOpenMessageCurrentActValue])));
    *(objs+index*numberOfObjs+positionActValue)       = new BVCanOpenMessagePositionActValue(  * ((BVCanOpenMessagePositionActValue*)(objectsSet->objs[ID_BVCanOpenMessagePositionActValue])));
    
    *(objs+index*numberOfObjs+initCWReset)            = new BVCanOpenMessageControlWord(       * ((BVCanOpenMessageControlWord*)(objectsSet->objs[ID_BVCanOpenMessageControlWord])));
 
    int cw = 0x06;
    (*(objs+index*numberOfObjs+initCW0))->prepareToSend( nodeID[index],BVCanOpenObject::write, &cw );
    cw = 0x07;
    (*(objs+index*numberOfObjs+initCW1))->prepareToSend( nodeID[index],BVCanOpenObject::write, &cw );
    cw = 0x0f;
    (*(objs+index*numberOfObjs+initCW2))->prepareToSend( nodeID[index],BVCanOpenObject::write, &cw );

    (*(objs+index*numberOfObjs+initSW0))->prepareToSend( nodeID[index],BVCanOpenObject::read, &cw );

    int vel=0;
    (*(objs+index*numberOfObjs+targetVelocity))->prepareToSend( nodeID[index],BVCanOpenObject::write, &vel );
    cw = 0x0f;
    (*(objs+index*numberOfObjs+CWAfterTargetVelocity))->prepareToSend( nodeID[index],BVCanOpenObject::write, &cw );
    int dummy=0;
    (*(objs+index*numberOfObjs+velSensorActValue))->prepareToSend( nodeID[index],BVCanOpenObject::read, &dummy );
    (*(objs+index*numberOfObjs+currentActValue))->prepareToSend( nodeID[index],BVCanOpenObject::read, &dummy );
    (*(objs+index*numberOfObjs+positionActValue))->prepareToSend( nodeID[index],BVCanOpenObject::read, &dummy );

    cw = 0x80;
    (*(objs+index*numberOfObjs+initCWReset))->prepareToSend( nodeID[index],BVCanOpenObject::write, &cw );

    eposState[index]=beforeStartState;
    nextObjToSendIndex[index]=idleMessage;
  }


  stateName[beforeStartState]= string("beforeStartState");
  stateName[getStatus]=string("getStatus"); 
  stateName[getStatusSent]=string("getStatusSent"); 
  stateName[switchOnDisabled]=string("switchOnDisabled"); 
  stateName[shutDownSent]=string("shutDownSent"); 
  stateName[readyToSwitchOn]=string("readyToSwitchOn"); 
  stateName[switchOnSent]=string("switchOnSent"); 
  stateName[switchedOn]=string("switchedOn"); 
  stateName[enabledOperationSent]=string("enabledOperationSent"); 
  stateName[operationEnabled]=string("operationEnabled"); 
  stateName[sentTargetVelocity]=string("sentTargetVelocity"); 
  stateName[recievedTargetVelocity]=string("recievedTargetVelocity"); 
  stateName[sentCwAfterTargetVelocity]=string("sentCwAfterTargetVelocity"); 
  stateName[recievedCwAfterTargetVelocity]=string("recievedCwAfterTargetVelocity"); 
  stateName[sentVelSensorActValue]=string("sentVelSensorActValue"); 
  stateName[recievedVelSensorActValue]=string("recievedVelSensorActValue"); 
  stateName[sentCurrentActValue]=string("sentCurrentActValue"); 
  stateName[recievedCurrentActValue]=string("recievedCurrentActValue"); 
  stateName[sentPositionActValue]=string("sentPositionActValue"); 
  stateName[idleState]=string("idleState"); 
  stateName[fault]=string("fault"); 
  stateName[sentFaultReset]=string("sentFaultReset");
  stateName[recievedFaultReset]=string("recievedFaultReset"); 
 

}
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -- getNextObjToSend -----------------------------------------------------
// @param1,  the index of the EPOS, 0,1,2
//           index 0 ---> nodeID[0]
//           index 1 ---> nodeID[1]
//           index 2 ---> nodeID[2]
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
void BVCanBusEposProtocol1::setToState(int index , enum enumEposState aEposState, enum enumCurrentMsgIndex anextMsgIndex){
  BV_ASSERT(0<=index && index<3);
  nextObjToSendIndex[index]= anextMsgIndex;
  eposState[index]=aEposState;
}
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// isSent
// @param1,  the index of the EPOS, 0,1,2
//           index 0 ---> nodeID[0]
//           index 1 ---> nodeID[1]
//           index 2 ---> nodeID[2] 
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
bool BVCanBusEposProtocol1::isSent( int index){

  if( eposState[index]==getStatusSent  ||
      eposState[index]==shutDownSent  ||
      eposState[index]==switchOnSent  ||
      eposState[index]==enabledOperationSent  ||
      eposState[index]==sentTargetVelocity  ||
      eposState[index]==sentCwAfterTargetVelocity  ||
      eposState[index]==sentVelSensorActValue  ||
      eposState[index]==sentCurrentActValue  ||
      eposState[index]==sentPositionActValue  ||
      eposState[index]==sentFaultReset ) return true;
  else return false;
}
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -- getNextObjToSend -----------------------------------------------------
// @param1,  the index of the EPOS, 0,1,2
//           index 0 ---> nodeID[0]
//           index 1 ---> nodeID[1]
//           index 2 ---> nodeID[2]
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
bool BVCanBusEposProtocol1::getNextObjToSend(int index , BVCanOpenObject** anObj){
  
  BV_ASSERT(0<=index && index<3);
  if(nextObjToSendIndex[index]==idleMessage)
    return false;
  else{
    if( nextObjToSendIndex[index]<0 || numberOfObjs<=nextObjToSendIndex[index]   ){
      BV_WARNING("BVCanBusEposProtocol1::getNextObjToSend index=" << index << " nextObjToSendIndex[index]=" << nextObjToSendIndex[index] << "  numberOfObjs=" << numberOfObjs );
      BV_ASSERT(0<=nextObjToSendIndex[index] && nextObjToSendIndex[index]<numberOfObjs);
    }
    *anObj = *( objs + index*numberOfObjs + nextObjToSendIndex[index]  );
    return true;
  }
}
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// -- getCanNodeID --------------------------------------------------------
// @param, an index , 0,1,2
// @return , the coresponding nodeID :0x02,0x04,0x08
//           index 0 ---> nodeID[0]
//           index 1 ---> nodeID[1]
//           index 2 ---> nodeID[2]
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
int BVCanBusEposProtocol1::getNodeID( int index ){
  BV_ASSERT( 0<=index && index<=2); 
  return nodeID[index];
}
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// -- getEposState --------------------------------------------------------
// @param1,  the index of the EPOS, 0,1,2
//           index 0 ---> nodeID[0]
//           index 1 ---> nodeID[1]
//           index 2 ---> nodeID[2]
// @return , thh current state of the state machine for
//           the EPOS corresponding to index
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
enum BVCanBusEposProtocol1::enumEposState BVCanBusEposProtocol1::getEposState(int index , string* aname){

  
  *aname = stateName[ eposState[index]  ];
  return eposState[index];
}
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// -- sentMessage ---------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
void BVCanBusEposProtocol1::sentMessage( BVCanOpenObject* obj ){

  int canNodeID = obj->getMsg().getCanID() - 0x600;
  if( canNodeID==nodeID[0] || canNodeID==nodeID[1] || canNodeID==nodeID[2] ){
  }
  else{
    BV_WARNING("canNodeID=" << canNodeID);
  } 
  BV_ASSERT( canNodeID==nodeID[0] || canNodeID==nodeID[1] || canNodeID==nodeID[2]);
  // ..................................................
  // ..................................................
  // get the index from the canNodeID
  // ..................................................
  // ..................................................
  int index;
  if     (canNodeID==nodeID[0]) index=0;
  else if(canNodeID==nodeID[1]) index=1;
  else if(canNodeID==nodeID[2]) index=2;
  else BV_ASSERT(0);

  switch(obj->getCanOpenObjectID()){
  case ID_BVCanOpenMessageControlWord:
  case ID_BVCanOpenMessageStatusWord:
  case ID_BVCanOpenMessageTargetVelocity:
  case ID_BVCanOpenMessageVelSensorActValue:
  case ID_BVCanOpenMessageCurrentActValue:
  case ID_BVCanOpenMessagePositionActValue:
    updateEposState(index , obj , BVCanOpenObject::pcToEpos);
    break;
  default:
    BV_WARNING("BVCanBusEposProtocol1::sentMessage unimplemeted message in the protocol canOpenObjectID=" << obj->getCanOpenObjectID());
    BV_ASSERT(0);
    break;
  } 
}

// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// -- recievedMessage -----------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
void BVCanBusEposProtocol1::recievedMessage( BVCanOpenObject* obj ){
  BVMessageTPCANMsg arecievedMessage=obj->getMsg();
  int senderID     = arecievedMessage.getSender();
  int recieverID   = arecievedMessage.getReciever();
  int messageId    = arecievedMessage.getID();
  int canID        = (int)( arecievedMessage.getCanID()   );
  int messasgeType = (int)( arecievedMessage.getMSGTYPE() );
  int length       = (int)( arecievedMessage.getLEN()     );
  int data0        = (int)( arecievedMessage.getDATA(0)   );
  int data1        = (int)( arecievedMessage.getDATA(1)   );
  int data2        = (int)( arecievedMessage.getDATA(2)   );
  int data3        = (int)( arecievedMessage.getDATA(3)   );
  int data4        = (int)( arecievedMessage.getDATA(4)   );
  int data5        = (int)( arecievedMessage.getDATA(5)   );
  int data6        = (int)( arecievedMessage.getDATA(6)   );
  int data7        = (int)( arecievedMessage.getDATA(7)   );

  int canNodeID = obj->getMsg().getCanID() - 0x580;

  if(canNodeID!=nodeID[0] && canNodeID!=nodeID[1] && canNodeID!=nodeID[2] ){
    BV_WARNING("BVCanBusEposProtocol1::recievedMessage unknown canID=" << obj->getMsg().getCanID());
  }
  else{
    // ..................................................
    // ..................................................
    // get the index from the canNodeID
    // ..................................................
    // ..................................................
    int index;
    if     (canNodeID==nodeID[0]) index=0;
    else if(canNodeID==nodeID[1]) index=1;
    else if(canNodeID==nodeID[2]) index=2;
    else BV_ASSERT(0);

    switch(obj->getCanOpenObjectID()){
    case ID_BVCanOpenMessageControlWord:
    case ID_BVCanOpenMessageStatusWord:
    case ID_BVCanOpenMessageTargetVelocity:
    case ID_BVCanOpenMessageVelSensorActValue:
    case ID_BVCanOpenMessageCurrentActValue:
    case ID_BVCanOpenMessagePositionActValue:{
      updateEposState(index , obj , BVCanOpenObject::eposToPc);
    }
      break;
    default:
      BV_WARNING("BVCanBusEposProtocol1::recievedMessage unimplemeted message in the protocol canID=" << obj->getMsg().getCanID() );
      BV_ASSERT(0);
      break;
    }
  }
}
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// -- updateEposState -----------------------------------------------------
// @param1,  the index of the EPOS, 0,1,2
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
void BVCanBusEposProtocol1::updateEposState( int index , BVCanOpenObject* obj , BVCanOpenObject::directionEnum adirection){
  // ..............................................................................
  // .. PC--->EPOS ................................................................
  // ..............................................................................
  if(adirection==BVCanOpenObject::pcToEpos){
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageControlWord........................................
    // ...................................................................
    // ...................................................................
    if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageControlWord ){
      if     (eposState[index]==switchOnDisabled )       eposState[index]=shutDownSent;
      else if(eposState[index]==readyToSwitchOn)         eposState[index]=switchOnSent; 
      else if(eposState[index]==switchedOn)              eposState[index]=enabledOperationSent; 
      else if(eposState[index]==recievedTargetVelocity)  eposState[index]=sentCwAfterTargetVelocity;
      else if(eposState[index]==fault){
        //BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState PC>EPOS[" << index << "] SENT:canObjectID=["<< obj->getName()<< "]=[initCWReset]:DONE...NEW eposState["<<index<<"]=["<<sentFaultReset<<"]=[sentFaultReset]");
        eposState[index]=sentFaultReset;
      }
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR PC>EPOS[" << index << "]  SENT canObjectID=["<< obj->getName()<< "] incompatible with current eposState["<<index<<"]=["<<stateName[eposState[index]]<<"]");
        BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageStatusWord.........................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageStatusWord ){
      if     (eposState[index]==getStatus )    eposState[index]=getStatusSent;
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR PC>EPOS[" << index << "]  SENT canObjectID=["<< obj->getName()<< "] incompatible with current eposState["<<index<<"]="<<stateName[eposState[index]]<<"]");
        BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageTargetVelocity.....................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageTargetVelocity ){
      if(eposState[index]==operationEnabled )             eposState[index]=sentTargetVelocity;
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR PC>EPOS[" << index << "]  SENT canObjectID=["<< obj->getName()<< "] incompatible with current eposState["<<index<<"]="<<stateName[eposState[index]]<<"]");
        BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageVelSensorActValue..................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageVelSensorActValue ){
      if(eposState[index]==recievedCwAfterTargetVelocity)  eposState[index]=sentVelSensorActValue;
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR PC>EPOS[" << index << "]  SENT canObjectID=["<< obj->getName()<< "] incompatible with current eposState["<<index<<"]=["<<stateName[eposState[index]]<<"]");
        BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageCurrentActValue.....................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageCurrentActValue ){
      if(eposState[index]==recievedVelSensorActValue )  eposState[index]=sentCurrentActValue;
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR PC>EPOS[" << index << "]  SENT canObjectID=["<< obj->getName()<< "] incompatible with current eposState["<<index<<"]=["<<stateName[eposState[index]]<<"]");
        BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessagePositionActValue...................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessagePositionActValue ){
      if(eposState[index]==recievedCurrentActValue )  eposState[index]=sentPositionActValue;
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR PC>EPOS[" << index << "]  SENT canObjectID=["<< obj->getName()<< "] incompatible with current eposState["<<index<<"]=["<<stateName[eposState[index]]<<"]");
        BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // U N K N O W N......................................................
    // ...................................................................
    // ...................................................................
    else{
      BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR PC>EPOS[" << index << "]  SENT canObjectID=["<< obj->getName()<< "] incompatible with current eposState["<<index<<"]=["<<stateName[eposState[index]]<<"]");
      BV_ASSERT(0);
    }
    nextObjToSendIndex[index]=idleMessage;
  }
  // ..............................................................................
  // ..............................................................................
  // ..............................................................................
  // ..............................................................................
  // ..............................................................................
  // ..............................................................................
  // ..............................................................................
  // ..............................................................................
  // .. EPOS--->PC ................................................................
  // ..............................................................................
  if(adirection==BVCanOpenObject::eposToPc){
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageControlWord........................................
    // ...................................................................
    // ...................................................................
    if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageControlWord ){
      if(eposState[index]==shutDownSent ){
        eposState[index]=getStatus; 
        nextObjToSendIndex[index]=initSW0;
      } 
      else if(eposState[index]==switchOnSent){
        eposState[index]=getStatus; 
        nextObjToSendIndex[index]=initSW0;
      }
      else if(eposState[index]==enabledOperationSent){
        eposState[index]=getStatus; 
        nextObjToSendIndex[index]=initSW0;
      }
      else if(eposState[index]==sentCwAfterTargetVelocity){
        eposState[index]=recievedCwAfterTargetVelocity; 
        nextObjToSendIndex[index]=velSensorActValue;
      }
      else if(eposState[index]==sentFaultReset){
        eposState[index]=getStatus; 
        nextObjToSendIndex[index]=initSW0;
      }
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR EPOS["<<index<<"]:RECIEVED canObjectID=["<<obj->getName()<<"]:IN:eposState["<<index<<"]=["<<stateName[eposState[index]]<< "]");
        //nextObjToSendIndex[index]=idleMessage;
        //BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageStatusWord.........................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageStatusWord ){
      if(eposState[index]==getStatusSent ){
        unsigned int statusWord = obj->getValU32();
        switch( statusWord & 0x417f  ){
        case 0x0000:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " BOOTUP");
          eposState[index]=getStatus;
          nextObjToSendIndex[index]=initSW0;
          break;
        case 0x0100:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " NOT READY TO SWITCH ON");
          eposState[index]=getStatus;
          nextObjToSendIndex[index]=initSW0;
          break;
        case 0x0140:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " SWITCH ON DISABLED");
          eposState[index]=switchOnDisabled;
          nextObjToSendIndex[index]=initCW0;
          break;
        case 0x0121:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " READY TO SWITCH ON");
          eposState[index]=readyToSwitchOn;
          nextObjToSendIndex[index]=initCW1;
          break;
        case 0x0123:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " SWITCHED ON");
          eposState[index]=switchedOn;
          nextObjToSendIndex[index]=initCW2;
          break;
        case 0x4123:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " REFRESH");
          eposState[index]=getStatus;
          nextObjToSendIndex[index]=initSW0;
          break;
        case 0x4133:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " MEASURE UNIT");
          eposState[index]=getStatus;
          nextObjToSendIndex[index]=initSW0;
          break;
        case 0x0137:
          //BV_WARNING("BVCanBusEposProtocol1::updateEposState index=" << index << " OPERATION ENABLED");
          eposState[index]=operationEnabled;
          nextObjToSendIndex[index]=targetVelocity;
          break;
        case 0x0117:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " QUICK STOP ACTIVE");
          break;
        case 0x010F:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " FAULT REACTION ACTIVE (DISABLED)");
          break;
        case 0x011F:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " FAULT REACTION ACTIVE (ENABLED)");
          break;
        case 0x0108:
          BV_DEBUGINFO4("BVCanBusEposProtocol1::updateEposState index=" << index << " FAULT");
          eposState[index]=fault;
          nextObjToSendIndex[index]=initCWReset;
          break;
        default:
          BV_WARNING("BVCanBusEposProtocol1::updateEposState index=" << index << " UNKNOWN statusWORD=" << hex << statusWord  << dec);
          break;
        }
      }
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR EPOS["<<index<<"]:RECIEVED:canObjectID=["<<obj->getName()<<"]:IN:eposState["<<index<<"]=["<<stateName[eposState[index]]<< "]");
        //nextObjToSendIndex[index]=idleMessage;
        //BV_ASSERT(0);
      }

    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageTargetVelocity.....................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageTargetVelocity){
      if(eposState[index]==sentTargetVelocity){
        eposState[index]=recievedTargetVelocity;  
        nextObjToSendIndex[index]=CWAfterTargetVelocity; 
        
      }
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR EPOS["<<index<<"]:RECIEVED:canObjectID=["<<obj->getName()<<"]:IN:eposState["<<index<<"]=["<<stateName[eposState[index]]<< "]");
        //nextObjToSendIndex[index]=idleMessage;
        //BV_ASSERT(0);
      } 
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageVelSensorActValue..................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageVelSensorActValue){
      if(eposState[index]==sentVelSensorActValue){
        eposState[index]=recievedVelSensorActValue;  
        nextObjToSendIndex[index]=currentActValue;
      }
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR EPOS["<<index<<"]:RECIEVED:canObjectID=["<<obj->getName()<<"]:IN:eposState["<<index<<"]=["<<stateName[eposState[index]]<< "]");
        //nextObjToSendIndex[index]=idleMessage;
        //BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessageCurrentActValue.....................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessageCurrentActValue){
      if(eposState[index]==sentCurrentActValue){
        eposState[index]=recievedCurrentActValue;
        nextObjToSendIndex[index]=positionActValue;  
      }
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR EPOS["<<index<<"]:RECIEVED:canObjectID=["<<obj->getName()<<"]:IN:eposState["<<index<<"]=["<<stateName[eposState[index]]<< "]");
	//nextObjToSendIndex[index]=idleMessage;
        //BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // BVCanOpenMessagePositionActValue...................................
    // ...................................................................
    // ...................................................................
    else if(obj->getCanOpenObjectID() == ID_BVCanOpenMessagePositionActValue){
      if(eposState[index]==sentPositionActValue){
        eposState[index]=getStatus;
        nextObjToSendIndex[index]=initSW0;  // 1 time
      }
      else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR EPOS["<<index<<"]:RECIEVED:canObjectID=["<<obj->getName()<<"]:IN:eposState["<<index<<"]=["<< stateName[eposState[index]]<< "]");
        //nextObjToSendIndex[index]=idleMessage;
        //BV_ASSERT(0);
      }
    }
    // ...................................................................
    // ...................................................................
    // U N K N O W N .....................................................
    // ...................................................................
    // ...................................................................
    else{
        BV_WARNING("BVCanBusEposProtocol1::updateEposState ERROR EPOS["<<index<<"]:RECIEVED:UNKNOWN canObjectID=["<<obj->getName()<<"]:IN:eposState["<<index<<"]=["<<stateName[eposState[index]] << "]");
        //nextObjToSendIndex[index]=idleMessage;
        //BV_ASSERT(0);
    }
    // ...................................................................
    // ...................................................................
    //....................................................................
    // ...................................................................
    // ...................................................................
  }
  // .....................................................................
  // .....................................................................
  // .....................................................................
}
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

