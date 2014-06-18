#ifndef BV_CAN_OPEN_MESSAGE_HANDLER
#define BV_CAN_OPEN_MESSAGE_HANDLER

#include "bvmessage.h"
#include "bvcanopenmessage.h"

class BVCanOpenMessageHandler{
public:
  BVCanOpenMessageHandler();

protected:
  /** a vector of podoubleers to the message handlers */
  void (BVCanOpenMessageHandler::*pToCanOpenMessageHandler[100])(BVMessageTPCANMsg *msg,double col);

  virtual void handlerBVCanOpenMessageSave(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageDeviceType(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageDeviceName(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageNodeID(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageCanBaudRate(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageVersion(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageMotorType(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageContinousCurrentLimit(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageOutputCurrentLimit(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessagePolePairNumber(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageMaximumSpeedInCurrentMode(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageThermalTimeConstantWinding(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageEncoderPulseNumber(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessagePositionSensorType(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageCurrenrRegulatorPGain(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageCurrenrRegulatorIGain(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageSpeedRegulatorPGain(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageSpeedRegulatorIGain(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessagePosRegulatorPGain(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessagePosRegulatorIGain(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessagePosRegulatorDGain(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageSetOperationMode(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageMaxProfileVelocity(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageProfileAcc(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageProfileDec(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageQuickStopDec(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageMotionProfileType(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageControlWord(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageTargetVelocity(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageStatusWord(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageVelSensorActValue(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageMisConfiguration(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageCurrentActValue(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageCobIDEmcy(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessagePositionActValue(BVMessageTPCANMsg *msg,double col);
  virtual void handlerBVCanOpenMessageVelModeSetValue(BVMessageTPCANMsg *msg,double col); 
};

#endif


