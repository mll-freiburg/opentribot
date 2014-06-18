#include "bvcanopenmessagehandler.h"

BVCanOpenMessageHandler::BVCanOpenMessageHandler(){
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageSave ]                       = &BVCanOpenMessageHandler::handlerBVCanOpenMessageSave;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageDeviceType ]                 = &BVCanOpenMessageHandler::handlerBVCanOpenMessageDeviceType;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageDeviceName ]                 = &BVCanOpenMessageHandler::handlerBVCanOpenMessageDeviceName;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageNodeID ]                     = &BVCanOpenMessageHandler::handlerBVCanOpenMessageNodeID;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageCanBaudRate ]                = &BVCanOpenMessageHandler::handlerBVCanOpenMessageCanBaudRate;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageVersion ]                    = &BVCanOpenMessageHandler::handlerBVCanOpenMessageVersion;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageMotorType ]                  = &BVCanOpenMessageHandler::handlerBVCanOpenMessageMotorType;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageContinousCurrentLimit ]      = &BVCanOpenMessageHandler::handlerBVCanOpenMessageContinousCurrentLimit;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageOutputCurrentLimit ]         = &BVCanOpenMessageHandler::handlerBVCanOpenMessageOutputCurrentLimit;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessagePolePairNumber ]             = &BVCanOpenMessageHandler::handlerBVCanOpenMessagePolePairNumber;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageMaximumSpeedInCurrentMode ]  = &BVCanOpenMessageHandler::handlerBVCanOpenMessageMaximumSpeedInCurrentMode;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageThermalTimeConstantWinding ] = &BVCanOpenMessageHandler::handlerBVCanOpenMessageThermalTimeConstantWinding;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageEncoderPulseNumber ]         = &BVCanOpenMessageHandler::handlerBVCanOpenMessageEncoderPulseNumber;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessagePositionSensorType ]         = &BVCanOpenMessageHandler::handlerBVCanOpenMessagePositionSensorType;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageCurrenrRegulatorPGain ]      = &BVCanOpenMessageHandler::handlerBVCanOpenMessageCurrenrRegulatorPGain;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageCurrenrRegulatorIGain ]      = &BVCanOpenMessageHandler::handlerBVCanOpenMessageCurrenrRegulatorIGain;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageSpeedRegulatorPGain ]        = &BVCanOpenMessageHandler::handlerBVCanOpenMessageSpeedRegulatorPGain;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageSpeedRegulatorIGain ]        = &BVCanOpenMessageHandler::handlerBVCanOpenMessageSpeedRegulatorIGain;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessagePosRegulatorPGain ]          = &BVCanOpenMessageHandler::handlerBVCanOpenMessagePosRegulatorPGain;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessagePosRegulatorIGain ]          = &BVCanOpenMessageHandler::handlerBVCanOpenMessagePosRegulatorIGain;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessagePosRegulatorDGain ]          = &BVCanOpenMessageHandler::handlerBVCanOpenMessagePosRegulatorDGain;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageSetOperationMode]            = &BVCanOpenMessageHandler::handlerBVCanOpenMessageSetOperationMode;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageMaxProfileVelocity]          = &BVCanOpenMessageHandler::handlerBVCanOpenMessageMaxProfileVelocity;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageProfileAcc]                  = &BVCanOpenMessageHandler::handlerBVCanOpenMessageProfileAcc;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageProfileDec]                  = &BVCanOpenMessageHandler::handlerBVCanOpenMessageProfileDec;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageQuickStopDec]                = &BVCanOpenMessageHandler::handlerBVCanOpenMessageQuickStopDec;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageMotionProfileType]           = &BVCanOpenMessageHandler::handlerBVCanOpenMessageMotionProfileType;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageControlWord]                 = &BVCanOpenMessageHandler::handlerBVCanOpenMessageControlWord;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageTargetVelocity]              = &BVCanOpenMessageHandler::handlerBVCanOpenMessageTargetVelocity;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageStatusWord]                  = &BVCanOpenMessageHandler::handlerBVCanOpenMessageStatusWord;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageVelSensorActValue]           = &BVCanOpenMessageHandler::handlerBVCanOpenMessageVelSensorActValue;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageMisConfiguration]            = &BVCanOpenMessageHandler::handlerBVCanOpenMessageMisConfiguration;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageCurrentActValue]             = &BVCanOpenMessageHandler::handlerBVCanOpenMessageCurrentActValue;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageCobIDEmcy]                   = &BVCanOpenMessageHandler::handlerBVCanOpenMessageCobIDEmcy;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessagePositionActValue]            = &BVCanOpenMessageHandler::handlerBVCanOpenMessagePositionActValue;
  pToCanOpenMessageHandler[ ID_BVCanOpenMessageVelModeSetValue ]            = &BVCanOpenMessageHandler::handlerBVCanOpenMessageVelModeSetValue;

}

void BVCanOpenMessageHandler::handlerBVCanOpenMessageSave(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageDeviceType(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageDeviceName(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageNodeID(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageCanBaudRate(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageVersion(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageMotorType(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageContinousCurrentLimit(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageOutputCurrentLimit(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessagePolePairNumber(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageMaximumSpeedInCurrentMode(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageThermalTimeConstantWinding(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageEncoderPulseNumber(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessagePositionSensorType(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageCurrenrRegulatorPGain(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageCurrenrRegulatorIGain(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageSpeedRegulatorPGain(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageSpeedRegulatorIGain(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessagePosRegulatorPGain(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessagePosRegulatorIGain(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessagePosRegulatorDGain(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageSetOperationMode(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageMaxProfileVelocity(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageProfileAcc(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageProfileDec(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageQuickStopDec(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageMotionProfileType(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageControlWord(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageTargetVelocity(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageStatusWord(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageVelSensorActValue(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageMisConfiguration(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageCurrentActValue(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageCobIDEmcy(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessagePositionActValue(BVMessageTPCANMsg *msg,double col){}
void BVCanOpenMessageHandler::handlerBVCanOpenMessageVelModeSetValue(BVMessageTPCANMsg *msg,double col){}
