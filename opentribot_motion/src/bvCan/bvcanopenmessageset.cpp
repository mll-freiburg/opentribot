#include "bvcanopenmessageset.h"
// ------------------------------------------------------------
// ------------------------------------------------------------
// -- ctor ----------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
BVCanOpenMessageSet::BVCanOpenMessageSet(){

  objs[ID_BVCanOpenMessageSave]                       =new BVCanOpenMessageSave(                      string("Save")            ,0x1010,0x01,BVCanOpenObject::UNSIGNED32 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageDeviceType]                 =new BVCanOpenMessageDeviceType(                string("DeviceType")      ,0x1000,0x00,BVCanOpenObject::UNSIGNED32 ,BVCanOpenObject::RO);
  objs[ID_BVCanOpenMessageDeviceName]               =new BVCanOpenMessageDeviceName(                string("deviceName")      ,0x1008,0x00,BVCanOpenObject::VISIBLE_STRING ,BVCanOpenObject::CONST);
  objs[ID_BVCanOpenMessageNodeID]                     =new BVCanOpenMessageNodeID(                    string("nodeID")          ,0x2000,0x00,BVCanOpenObject::UNSIGNED8  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageCanBaudRate]                =new BVCanOpenMessageCanBaudRate(               string("canBaudRate")     ,0x2001,0x00,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageVersion]                    =new BVCanOpenMessageVersion(                   string("version")         ,0x2003,0x01,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RO);
  objs[ID_BVCanOpenMessageMotorType]                  =new BVCanOpenMessageMotorType(                 string("motorType")       ,0x6402,0x00,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageContinousCurrentLimit]      =new BVCanOpenMessageContinousCurrentLimit(     string("contILimit")      ,0x6410,0x01,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageOutputCurrentLimit]         =new BVCanOpenMessageOutputCurrentLimit(        string("outILimit")       ,0x6410,0x02,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessagePolePairNumber]             =new BVCanOpenMessagePolePairNumber(            string("polePairNr")      ,0x6410,0x03,BVCanOpenObject::UNSIGNED8  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageMaximumSpeedInCurrentMode]  =new BVCanOpenMessageMaximumSpeedInCurrentMode( string("maxSpeedinIMode") ,0x6410,0x04,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageThermalTimeConstantWinding] =new BVCanOpenMessageThermalTimeConstantWinding(string("TTimeCte")        ,0x6410,0x05,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageEncoderPulseNumber]         =new BVCanOpenMessageEncoderPulseNumber(        string("encoderPulseNr")  ,0x2210,0x01,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessagePositionSensorType]         =new BVCanOpenMessagePositionSensorType(        string("posSensorType")   ,0x2210,0x02,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageCurrenrRegulatorPGain]      =new BVCanOpenMessageCurrenrRegulatorPGain(     string("I Ctrl P" )       ,0x60F6,0x01,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageCurrenrRegulatorIGain]      =new BVCanOpenMessageCurrenrRegulatorIGain(     string("I Ctrl I")        ,0x60F6,0x02,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageSpeedRegulatorPGain]        =new BVCanOpenMessageSpeedRegulatorPGain(       string("Vel Ctrl P")      ,0x60F9,0x01,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageSpeedRegulatorIGain]        =new BVCanOpenMessageSpeedRegulatorIGain(       string("Vel Ctrl I")      ,0x60F9,0x02,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessagePosRegulatorPGain]          =new BVCanOpenMessagePosRegulatorPGain(         string("Pos Ctrl P")      ,0x60FB,0x01,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessagePosRegulatorIGain]          =new BVCanOpenMessagePosRegulatorIGain(         string("Pos Ctrl I")      ,0x60FB,0x02,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessagePosRegulatorDGain]          =new BVCanOpenMessagePosRegulatorDGain(         string("Pos Ctrl D")      ,0x60FB,0x03,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageSetOperationMode]           =new BVCanOpenMessageSetOperationMode(          string("OP Mode")         ,0x6060,0x00,BVCanOpenObject::INTEGER8   ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageMaxProfileVelocity]         =new BVCanOpenMessageMaxProfileVelocity(        string("Max Prof V")      ,0x607F,0x00,BVCanOpenObject::UNSIGNED32 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageProfileAcc]                 =new BVCanOpenMessageProfileAcc(                string("Prof Acc")        ,0x6083,0x00,BVCanOpenObject::UNSIGNED32 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageProfileDec]                 =new BVCanOpenMessageProfileDec(                string("Prof Dec")        ,0x6084,0x00,BVCanOpenObject::UNSIGNED32 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageQuickStopDec]               =new BVCanOpenMessageQuickStopDec(              string("Quick S Dec")     ,0x6085,0x00,BVCanOpenObject::UNSIGNED32 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageMotionProfileType]          =new BVCanOpenMessageMotionProfileType(         string("Mot Prof Typ")    ,0x6086,0x00,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageControlWord]                =new BVCanOpenMessageControlWord(               string("Ctrl W")          ,0x6040,0x00,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageTargetVelocity]             =new BVCanOpenMessageTargetVelocity(            string("Targ V")          ,0x60FF,0x00,BVCanOpenObject::INTEGER32  ,BVCanOpenObject::RW);
  //objs[ID_BVCanOpenMessageTargetVelocity]             =new BVCanOpenMessageTargetVelocity(            string("Targ V")          ,0x206B,0x00,BVCanOpenObject::INTEGER32  ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageStatusWord]                 =new BVCanOpenMessageStatusWord(                string("Status W")        ,0x6041,0x00,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RO);
  objs[ID_BVCanOpenMessageVelSensorActValue]          =new BVCanOpenMessageVelSensorActValue(         string("V Actual")        ,0x6069,0x00,BVCanOpenObject::INTEGER32  ,BVCanOpenObject::RO);
  //objs[ID_BVCanOpenMessageVelSensorActValue]          =new BVCanOpenMessageVelSensorActValue(         string("V Actual")        ,0x606C,0x00,BVCanOpenObject::INTEGER32  ,BVCanOpenObject::RO);
  objs[ID_BVCanOpenMessageMisConfiguration]           =new BVCanOpenMessageMisConfiguration(          string("Mis Conf")        ,0x2008,0x00,BVCanOpenObject::UNSIGNED16 ,BVCanOpenObject::RW);
  objs[ID_BVCanOpenMessageCurrentActValue]            =new BVCanOpenMessageCurrentActValue(           string("I Actual")        ,0x6078,0x00,BVCanOpenObject::INTEGER16  ,BVCanOpenObject::RO);
  objs[ID_BVCanOpenMessageCobIDEmcy]                  =new BVCanOpenMessageCobIDEmcy(                 string("COB ID EMCY")     ,0x1014,0x00,BVCanOpenObject::UNSIGNED32 ,BVCanOpenObject::RO);
  objs[ID_BVCanOpenMessagePositionActValue]           =new BVCanOpenMessagePositionActValue(          string("Pos Act")         ,0x6064,0x00,BVCanOpenObject::INTEGER32  ,BVCanOpenObject::RO);
  objs[ID_BVCanOpenMessageVelModeSetValue]            =new BVCanOpenMessageVelModeSetValue(           string("Vel Set")         ,0x206B,0x00,BVCanOpenObject::INTEGER32  ,BVCanOpenObject::RW);

  vecSize=36;
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// -- dtor ----------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
// FIXME
BVCanOpenMessageSet::~BVCanOpenMessageSet(){
  // delete the allocated objects
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// -- size ----------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
int BVCanOpenMessageSet::size(){ return vecSize; }
// ------------------------------------------------------------
// ------------------------------------------------------------
// -- getNameOfObject -----------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
bool BVCanOpenMessageSet::getNameOfObject(int anIndex , string* aname){
  for(int i=0;i<vecSize;i++){
    if( objs[i]->index == anIndex ){
      *aname = objs[i]->getName();
      return true;
    }
  }
  return false;
  
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// -- getCanOpenObjectID --------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
int BVCanOpenMessageSet::getCanOpenObjectID( BVMessageTPCANMsg *msg){

  int senderID = msg->getSender();
  int recieverID = msg->getReciever();
  int messageId = msg->getID();
  int canID = (int)msg->getCanID();
  int messasgeType = (int)msg->getMSGTYPE();
  int length = (int)msg->getLEN();
  int data0 = (int)msg->getDATA(0);
  int data1 = (int)msg->getDATA(1);
  int data2 = (int)msg->getDATA(2);
  int data3 = (int)msg->getDATA(3);
  int data4 = (int)msg->getDATA(4);
  int data5 = (int)msg->getDATA(5);
  int data6 = (int)msg->getDATA(6);
  int data7 = (int)msg->getDATA(7);

  int index   = (data2 << 8) + data1;
  int d2 = ( data2 << 8  ) ;
  int d3 = d2 & 0xFF00;
  int d4 = d3 + data1;
  if( index!=d4  ){
    BV_WARNING("BVCanOpenMessageSet::getCanOpenObjectID data1=" <<data1 << " data2=" << data2 << " index=" << index << " d4=" << d4  );
    BV_ASSERT( index==d4  );
  } 

  int subIndex= data3;
  int value8  = data4;
  int value16 = (data5 << 8) + data4;
  int value32 = (data7 << 24) + + (data6 << 16) +(data5 << 8) + data4;

  int canMessageID;
  // ......................................
  // ......................................
  // ......................................
  // .........objectsSet[BVCanOpenObjectsSet::.............................
  // ......................................
  if     (index == objs[ID_BVCanOpenMessageSave]->index)                                           canMessageID=ID_BVCanOpenMessageSave;
  else if(index == objs[ID_BVCanOpenMessageDeviceType]->index)                                     canMessageID=ID_BVCanOpenMessageDeviceType;
  else if(index == objs[ID_BVCanOpenMessageDeviceName]->index)                                     canMessageID=ID_BVCanOpenMessageDeviceName;
  else if(index == objs[ID_BVCanOpenMessageNodeID]->index)                                         canMessageID=ID_BVCanOpenMessageNodeID;
  else if(index == objs[ID_BVCanOpenMessageCanBaudRate]->index)                                    canMessageID=ID_BVCanOpenMessageCanBaudRate;
  else if(index == objs[ID_BVCanOpenMessageVersion]->index)                                        canMessageID=ID_BVCanOpenMessageVersion;
  else if(index == objs[ID_BVCanOpenMessageMotorType]->index)                                      canMessageID=ID_BVCanOpenMessageMotorType;
  else if(index == objs[ID_BVCanOpenMessageContinousCurrentLimit]->index      && subIndex==0x01)   canMessageID=ID_BVCanOpenMessageContinousCurrentLimit;
  else if(index == objs[ID_BVCanOpenMessageOutputCurrentLimit]->index         && subIndex==0x02)   canMessageID=ID_BVCanOpenMessageOutputCurrentLimit;
  else if(index == objs[ID_BVCanOpenMessagePolePairNumber]->index             && subIndex==0x03)   canMessageID=ID_BVCanOpenMessagePolePairNumber;
  else if(index == objs[ID_BVCanOpenMessageMaximumSpeedInCurrentMode]->index  && subIndex==0x04)   canMessageID=ID_BVCanOpenMessageMaximumSpeedInCurrentMode;
  else if(index == objs[ID_BVCanOpenMessageThermalTimeConstantWinding]->index && subIndex==0x05)   canMessageID=ID_BVCanOpenMessageThermalTimeConstantWinding;
  else if(index == objs[ID_BVCanOpenMessageEncoderPulseNumber]->index         && subIndex==0x01)   canMessageID=ID_BVCanOpenMessageEncoderPulseNumber;
  else if(index == objs[ID_BVCanOpenMessagePositionSensorType]->index         && subIndex==0x02)   canMessageID=ID_BVCanOpenMessagePositionSensorType;
  else if(index == objs[ID_BVCanOpenMessageCurrenrRegulatorPGain]->index      && subIndex==0x01)   canMessageID=ID_BVCanOpenMessageCurrenrRegulatorPGain;
  else if(index == objs[ID_BVCanOpenMessageCurrenrRegulatorIGain]->index      && subIndex==0x02)   canMessageID=ID_BVCanOpenMessageCurrenrRegulatorIGain;
  else if(index == objs[ID_BVCanOpenMessageSpeedRegulatorPGain]->index        && subIndex==0x01)   canMessageID=ID_BVCanOpenMessageSpeedRegulatorPGain;
  else if(index == objs[ID_BVCanOpenMessageSpeedRegulatorIGain]->index        && subIndex==0x02)   canMessageID=ID_BVCanOpenMessageSpeedRegulatorIGain;
  else if(index == objs[ID_BVCanOpenMessagePosRegulatorPGain]->index          && subIndex==0x01)   canMessageID=ID_BVCanOpenMessagePosRegulatorPGain;
  else if(index == objs[ID_BVCanOpenMessagePosRegulatorIGain]->index          && subIndex==0x02)   canMessageID=ID_BVCanOpenMessagePosRegulatorIGain;
  else if(index == objs[ID_BVCanOpenMessagePosRegulatorDGain]->index          && subIndex==0x03)   canMessageID=ID_BVCanOpenMessagePosRegulatorDGain;
  else if(index == objs[ID_BVCanOpenMessageSetOperationMode]->index )                              canMessageID=ID_BVCanOpenMessageSetOperationMode;
  else if(index == objs[ID_BVCanOpenMessageMaxProfileVelocity]->index )                            canMessageID=ID_BVCanOpenMessageMaxProfileVelocity;
  else if(index == objs[ID_BVCanOpenMessageProfileAcc]->index )                                    canMessageID=ID_BVCanOpenMessageProfileAcc;
  else if(index == objs[ID_BVCanOpenMessageProfileDec]->index )                                    canMessageID=ID_BVCanOpenMessageProfileDec;
  else if(index == objs[ID_BVCanOpenMessageQuickStopDec]->index )                                  canMessageID=ID_BVCanOpenMessageQuickStopDec;
  else if(index == objs[ID_BVCanOpenMessageMotionProfileType]->index )                             canMessageID=ID_BVCanOpenMessageMotionProfileType;
  else if(index == objs[ID_BVCanOpenMessageControlWord]->index )                                   canMessageID=ID_BVCanOpenMessageControlWord;
  else if(index == objs[ID_BVCanOpenMessageTargetVelocity]->index )                                canMessageID=ID_BVCanOpenMessageTargetVelocity;
  else if(index == objs[ID_BVCanOpenMessageStatusWord]->index )                                    canMessageID=ID_BVCanOpenMessageStatusWord;
  else if(index == objs[ID_BVCanOpenMessageVelSensorActValue]->index )                             canMessageID=ID_BVCanOpenMessageVelSensorActValue;
  else if(index == objs[ID_BVCanOpenMessageMisConfiguration]->index )                              canMessageID=ID_BVCanOpenMessageMisConfiguration;
  else if(index == objs[ID_BVCanOpenMessageCurrentActValue]->index )                               canMessageID=ID_BVCanOpenMessageCurrentActValue;
  else if(index == objs[ID_BVCanOpenMessageCobIDEmcy]->index )                                     canMessageID=ID_BVCanOpenMessageCobIDEmcy;
  else if(index == objs[ID_BVCanOpenMessagePositionActValue]->index )                              canMessageID=ID_BVCanOpenMessagePositionActValue;
  else if(index == objs[ID_BVCanOpenMessageVelModeSetValue]->index )                               canMessageID=ID_BVCanOpenMessageVelModeSetValue;
  else{
    BV_WARNING("BVCanOpenMessageSet::getCanOpenObject unknown index=0x" << hex << index <<  dec);
    canMessageID=-1;
  }
  // ......................................
  // ......................................
  // ......................................
  return canMessageID; 
}
// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------
// ------------------------------------------------------------



