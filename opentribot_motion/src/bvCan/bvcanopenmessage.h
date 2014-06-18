#ifndef BV_CAN_OPEN_MESSAGE
#define BV_CAN_OPEN_MESSAGE

#include "bvcanopenobject.h"

using namespace std;

#define ID_BVCanOpenMessageSave                        0
#define ID_BVCanOpenMessageDeviceType                  1
#define ID_BVCanOpenMessageDeviceName                  2
#define ID_BVCanOpenMessageNodeID                      3
#define ID_BVCanOpenMessageCanBaudRate                 4
#define ID_BVCanOpenMessageVersion                     5
#define ID_BVCanOpenMessageMotorType                   6
#define ID_BVCanOpenMessageContinousCurrentLimit       7
#define ID_BVCanOpenMessageOutputCurrentLimit          8
#define ID_BVCanOpenMessagePolePairNumber              9
#define ID_BVCanOpenMessageMaximumSpeedInCurrentMode   10
#define ID_BVCanOpenMessageThermalTimeConstantWinding  11
#define ID_BVCanOpenMessageEncoderPulseNumber          12
#define ID_BVCanOpenMessagePositionSensorType          13
#define ID_BVCanOpenMessageCurrenrRegulatorPGain       14
#define ID_BVCanOpenMessageCurrenrRegulatorIGain       15
#define ID_BVCanOpenMessageSpeedRegulatorPGain         16
#define ID_BVCanOpenMessageSpeedRegulatorIGain         17
#define ID_BVCanOpenMessagePosRegulatorPGain           18
#define ID_BVCanOpenMessagePosRegulatorIGain           19
#define ID_BVCanOpenMessagePosRegulatorDGain           20
#define ID_BVCanOpenMessageSetOperationMode            21
#define ID_BVCanOpenMessageMaxProfileVelocity          22
#define ID_BVCanOpenMessageProfileAcc                  23
#define ID_BVCanOpenMessageProfileDec                  24
#define ID_BVCanOpenMessageQuickStopDec                25
#define ID_BVCanOpenMessageMotionProfileType           26
#define ID_BVCanOpenMessageControlWord                 27
#define ID_BVCanOpenMessageTargetVelocity              28 
#define ID_BVCanOpenMessageStatusWord                  29
#define ID_BVCanOpenMessageVelSensorActValue           30
#define ID_BVCanOpenMessageMisConfiguration            31
#define ID_BVCanOpenMessageCurrentActValue             32
#define ID_BVCanOpenMessageCobIDEmcy                   33
#define ID_BVCanOpenMessagePositionActValue            34
#define ID_BVCanOpenMessageVelModeSetValue             35

#define MAKE_CAN_OPEN_MASSAGE_CLASS(MESSAGE_NAME) \
class MESSAGE_NAME : public BVCanOpenObject{ \
public: \
MESSAGE_NAME( string aName,int anIndex,int aSubIndex,enum typeEnum aType,enum accessEnum anAccess)\
 : BVCanOpenObject(aName,anIndex,aSubIndex,aType,anAccess) \
{\
canOpenObjectID = ID_##MESSAGE_NAME;\
} \
};

MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageSave)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageDeviceType)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageDeviceName)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageNodeID)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageCanBaudRate)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageVersion)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageMotorType)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageContinousCurrentLimit)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageOutputCurrentLimit)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessagePolePairNumber)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageMaximumSpeedInCurrentMode)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageThermalTimeConstantWinding)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageEncoderPulseNumber)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessagePositionSensorType)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageCurrenrRegulatorPGain)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageCurrenrRegulatorIGain)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageSpeedRegulatorPGain)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageSpeedRegulatorIGain)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessagePosRegulatorPGain)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessagePosRegulatorIGain)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessagePosRegulatorDGain)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageSetOperationMode)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageMaxProfileVelocity)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageProfileAcc)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageProfileDec)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageQuickStopDec)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageMotionProfileType)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageControlWord)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageTargetVelocity)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageStatusWord)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageVelSensorActValue)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageMisConfiguration)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageCurrentActValue)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageCobIDEmcy)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessagePositionActValue)
MAKE_CAN_OPEN_MASSAGE_CLASS(BVCanOpenMessageVelModeSetValue)

#endif 
