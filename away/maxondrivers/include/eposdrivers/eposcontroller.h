// Copyright (C) 2009 Rosen Diankov (rdiankov@cs.cmu.edu)
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef EPOS_CONTROLLER
#define EPOS_CONTROLLER

#include <openrave_robot_control/openravecontroller.h>

#include <cstdio>
#include <cstring>
#include <ntcan.h>
#include <unistd.h>
#include <cstdlib>
#include <stdint.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <string>

using namespace std;
using namespace openrave_robot_control;

/// CAN controller for a set of EPOS motor devices
class EPOSController : public OpenRAVEController
{
    class XMLMotorData : public XMLReadable
    {
    public:
        struct MOTORPARAMETERS
        {
            MOTORPARAMETERS() : transmissionratio(1),torquetocurrent(0),staticfriction(0),viscousfriction(0) {}

            dReal transmissionratio;
            /// multiplier to convert torque to current
            dReal torquetocurrent;
            dReal staticfriction, viscousfriction;
            dReal maxrpmspeed;
        };
        struct JOINTPARAMETERS
        {
            JOINTPARAMETERS() : pgain(0),igain(0),dgain(0),maxintegral(1e8),homingorder(0),homingtorque(0)
            {
                motortransmission[0] = motortransmission[1] = 0;
            }

            dReal pgain,igain,dgain;
            dReal maxintegral; // max integral term for pid 

            /// if positive, will go towards positive limit, otherwise negative limit, if 0, does not move.
            /// the magnitude determines the calibration order
            dReal homingorder, homingtorque;
            string motornames[2];
            dReal motortransmission[2];
        };

        XMLMotorData() : XMLReadable("motordata") {}
        map<string,MOTORPARAMETERS> _motors;
        map<string,JOINTPARAMETERS> _joints;
    };

    class XMLMotorDataReader : public BaseXMLReader
    {
    public:
        XMLMotorDataReader(XMLMotorData* motordata, const char **atts) {
            _motordata = motordata;
            if( _motordata == NULL )
                _motordata = new XMLMotorData();
            _itmotor = _motordata->_motors.end();
            _itjoint = _motordata->_joints.end();
        }
        virtual ~XMLMotorDataReader() { delete _motordata; }
        
        void* Release() { XMLMotorData* temp = _motordata; _motordata = NULL; return temp; }

        virtual void startElement(void *ctx, const char *name, const char **atts)
        {
            if( _itmotor != _motordata->_motors.end() ) {
            }
            else if( _itjoint != _motordata->_joints.end() ) {
            }
            else if( strcmp((const char*)name, "motor") == 0 ) {
                string name;
                for (int i = 0;(atts[i] != NULL);i+=2) {
                    if( strcmp(atts[i], "name") == 0 )
                        name = atts[i+1];
                }
                _itmotor = _motordata->_motors.insert(make_pair(name,XMLMotorData::MOTORPARAMETERS())).first;
            }
            else if( strcmp((const char*)name, "joint") == 0 ) {
                string name;
                for (int i = 0;(atts[i] != NULL);i+=2) {
                    if( strcmp(atts[i], "name") == 0 )
                        name = atts[i+1];
                }
                _itjoint = _motordata->_joints.insert(make_pair(name,XMLMotorData::JOINTPARAMETERS())).first;
            }
        }
        virtual bool endElement(void *ctx, const char *name)
        {
            if( _itmotor != _motordata->_motors.end() ) {
                if( strcmp((const char*)name, "motor") == 0 )
                    _itmotor = _motordata->_motors.end();
                else if( strcmp((const char*)name, "transmissionratio") == 0 )
                    ss >> _itmotor->second.transmissionratio;
                else if( strcmp((const char*)name, "staticfriction") == 0 )
                    ss >> _itmotor->second.staticfriction;
                else if( strcmp((const char*)name, "viscousfriction") == 0 )
                    ss >> _itmotor->second.viscousfriction;
                else if( strcmp((const char*)name, "torquetocurrent") == 0 )
                    ss >> _itmotor->second.torquetocurrent;
                else if( strcmp((const char*)name, "maxrpmspeed") == 0 )
                    ss >> _itmotor->second.maxrpmspeed;
            }
            else if( _itjoint != _motordata->_joints.end() ) {
                if( strcmp((const char*)name, "joint") == 0 )
                    _itjoint = _motordata->_joints.end();
                else if( strcmp((const char*)name, "pgain") == 0 )
                    ss >> _itjoint->second.pgain;
                else if( strcmp((const char*)name, "igain") == 0 )
                    ss >> _itjoint->second.igain;
                else if( strcmp((const char*)name, "dgain") == 0 )
                    ss >> _itjoint->second.dgain;
                else if( strcmp((const char*)name, "homing") == 0 )
                    ss >> _itjoint->second.homingorder >> _itjoint->second.homingtorque;
                else if( strcmp((const char*)name, "motor") == 0 )
                    ss >> _itjoint->second.motornames[0] >> _itjoint->second.motortransmission[0];
                else if( strcmp((const char*)name, "maxintegral") == 0 )
                    ss >> _itjoint->second.maxintegral;
                else if( strcmp((const char*)name, "diffmotor") == 0 )
                    ss >> _itjoint->second.motornames[0] >> _itjoint->second.motornames[1]
                       >> _itjoint->second.motortransmission[0] >> _itjoint->second.motortransmission[1];
            }
            else if( strcmp((const char*)name, "motordata") == 0 )
                return true;
            else
                ROS_ERROR("unknown field %s", name);
            
            if( !ss )
                ROS_ERROR("XMLMotorDataReader error parsing %s", name);
            
            return false;
        }
        
        virtual void characters(void *ctx, const char *ch, int len)
        {
            if( len > 0 ) {
                ss.clear();
                ss.str(string(ch, len));
            }
            else
                ss.str(""); // reset
        }

    protected:
        XMLMotorData* _motordata;
        map<string,XMLMotorData::MOTORPARAMETERS>::iterator _itmotor;
        map<string,XMLMotorData::JOINTPARAMETERS>::iterator _itjoint;
        stringstream ss;
    };

    static BaseXMLReader* CreateMotorDataXMLReader(InterfaceBase* pinterface, const char **atts)
    {
        return new XMLMotorDataReader(NULL, atts);
    }

protected:
    class CAN
    {
    public:
        NTCAN_HANDLE _canhandle;
        CMSG _canbuffer[16]; // internal receive buffer
        int _iCanCurMsgIndex;
        int _nCanBuffered; 
        int _iCanControl, _iCanMessage, _iCanMessageInfo, _iCanStatus;
        bool _bCanInit;

        CAN() : _bCanInit(false) {}
        ~CAN() {
            CanDestroy();
        }

        bool CanInit(int port, int speed)
        {
            if (canOpen(port,0,2000,2000,5,5, &_canhandle)!=NTCAN_SUCCESS) {
                ROS_ERROR("CAN device failed to open");
                return false;
            }
    
            if( speed == 250 ) {
                if (canSetBaudrate(_canhandle, 4)!=NTCAN_SUCCESS) {
                    ROS_ERROR("set baudrate to 250k failed");
                    return false;
                }
            }
            else if( speed == 500  ) {
                if (canSetBaudrate(_canhandle, 2)!=NTCAN_SUCCESS) {
                    ROS_ERROR("set baudrate to 1M failed");
                    return false;
                }
            }
            else if( speed == 1000  ) {
                if (canSetBaudrate(_canhandle, 0)!=NTCAN_SUCCESS) {
                    fprintf(stderr,"set baudrate to 1M failed\n");
                    return false;
                }
            }
            else return false;

            // Add all id's ??
            for (int i=0; i<=0x7FF; i++) {
                if (canIdAdd(_canhandle,i)!=NTCAN_SUCCESS) {
                    fprintf(stderr,"failed to add %d id\n",i);
                    return false;
                }
            }

            memset(_canbuffer, 0, sizeof(_canbuffer));
            _iCanCurMsgIndex = 0;
            _nCanBuffered = 0;
            _iCanControl = _iCanMessage = _iCanMessageInfo = _iCanStatus = -1;
            _bCanInit = true;
            return true;
        }

        bool CanGetMsg(CMSG& msg)
        {
            if( _iCanCurMsgIndex < _nCanBuffered ) {
                msg = _canbuffer[_iCanCurMsgIndex++];
            }
            else {
                _iCanCurMsgIndex = 0;
                _nCanBuffered = 0;
                int len=sizeof(_canbuffer)/sizeof(_canbuffer[0]);
                int err = canTake(_canhandle, _canbuffer, &len);
                if(err!=NTCAN_SUCCESS ) {
                    fprintf(stderr,"canTake error 0x%x!\n",err);
                    return false;
                }
        
                if( len <= 0 )
                    return false;
        
                _nCanBuffered = len;
                msg = _canbuffer[_iCanCurMsgIndex++];
            }

            return true;
        }

        bool CanSendRaw(int id, int len, int rtr, void* pdata)
        {
            CMSG msg;
            memset(&msg, 0, sizeof(msg));    
            msg.id = id;
            msg.len = len;
            if(rtr)
                msg.len |= NTCAN_RTR;
            memcpy(msg.data,pdata,len);
            int err;
            for(int iter = 0; iter < 4; ++iter) {
                int size = 1;
                err = canSend(_canhandle, &msg, &size);
                if( err == NTCAN_SUCCESS )
                    return true;
                usleep(10);
            }
            fprintf(stderr,"CanSendRaw failed (err=0x%x) msg = 0x%x\n", err,id);
            return false;
        }

        void CanDestroy()
        {
            if( !_bCanInit )
                return;
    
            if (canClose(_canhandle)!=NTCAN_SUCCESS)
                fprintf(stderr,"failed to close CAN bus\n");
            _bCanInit = false;
        }
    };

    enum AckCode
    {
        AC_OK =      0x4f,  ///< EPOS answer code for <em>all fine</em>
        AC_FAIL =   0x46,  ///< EPOS answer code to indicate a <em>failure</em>
        AC_ANS =    0x00,  ///< EPOS code to indicate an answer <em>frame</em>
    };

    /// CANopen defined error codes
    enum ErrorCode
    {
        EC_NoError=0x00000000,
        EC_ToggleError = 0x05030000,
        EC_SDOTimeOut = 0x05040000,
        EC_ClientServerSpecifierError = 0x05040001,
        EC_OutOfMemoryError = 0x05040005,
        EC_AccessError = 0x06010000,
        EC_WriteOnly=0x06010001,
        EC_ReadOnly=0x0601002,
        EC_ObjectDoesNotExist=0x06020000,
        EC_PDOMappingError = 0x06040041,
        EC_PDOLengthError = 0x06040042,
        EC_GeneralParameterError = 0x06040043,
        EC_GeneralInternIncompatibility = 0x06040047,
        EC_HardwareError = 0x06060000,
        EC_ServiceParameterError = 0x06070010,
        EC_ServiceParameterTooLong = 0x06070012,
        EC_ServiceParameterTooShort = 0x06070013,
        EC_ObjectSubIndexError       =0x06090011,
        EC_ValueRangeError       =0x06090030,
        EC_ValueTooHigh       =0x06090031,
        EC_ValueTooLow =0x06090032,
        EC_MaxLessMinError        =0x06090036,
        EC_GeneralError = 0x08000000,
        EC_TransferStoreError = 0x08000020,
        EC_LocalControlError = 0x08000021,
        EC_WrongDeviceState = 0x08000022,
        EC_WrongNMTState      =0x0f00ffc0,
        EC_IllegalCommandError         =0x0f00ffbf,
        EC_PasswordError        =0x0f00ffbe,
        EC_ErrorServiceMode         =0x0f00ffbc,
        EC_ErrorCANId        =0x0f00ffb9,
    };

    /// correlated with the error register
    enum ErrorDeviceCode {
        EDC_NoError = 0x0000,
        EDC_GenericError = 0x1000,
        EDC_OverCurrentError = 0x2310,
        EDC_OverVoltageError = 0x3210,
        EDC_UnderVoltage = 0x3220,
        EDC_OverTemperature = 0x4210,
        EDC_SupplyVoltageLow = 0x5113,
        EDC_InternalSoftwareError = 0x6100,
        EDC_SoftwareParameterError = 0x6320,
        EDC_SensorPositionError = 0x7320,
        EDC_CANOverrunErrorLost = 0x8110,
        EDC_CANOverrunError = 0x8111,
        EDC_CANPassiveModeError = 0x8120,
        EDC_CANLifeGuardError = 0x8130,
        EDC_CANTransmitCOBIDCollision = 0x8150,
        EDC_CANBusOff = 0x81fd,
        EDC_CANRxQueueOverrun = 0x81fe,
        EDC_CANTxQueueOverrun = 0x81ff,
        EDC_CANPDOLengthError = 0x8210,
        EDC_FollowingError = 0x8611,
        EDC_HallSensorError = 0xff01,
        EDC_IndexProcessingError = 0xff02,
        EDC_EncoderResolutionError = 0xff03,
        EDC_HallsensorNotFound = 0xff04,
        EDC_NegativeLimitError = 0xff06,
        EDC_PositiveLimitError = 0xff07,
        EDC_HallAngleDetectionError = 0xff08,
        EDC_SoftwarePositionLimit = 0xff09,
        EDC_PositionSensorBreach = 0xff0a,
        EDC_SystemOverloaded = 0xff0b,
    };
            
    // bits 15-0 is index, bits 23-16 is sub-index
    enum ObjectName {
        ON_ErrorRegister=0x001001,
        ON_ErrorHistoryNum=0x001003,
        ON_ErrorHistory1=0x011003,
        ON_ErrorHistory2=0x021003,
        ON_ErrorHistory3=0x031003,
        ON_ErrorHistory4=0x041003,
        ON_ErrorHistory5=0x051003,
        ON_SYNC=0x001005,
        ON_MiscellaneousConfiguration = 0x002008,
        ON_ReceivePDO1Parameter = 0x011400,
        ON_ReceivePDO2Parameter = 0x011401,
        ON_ReceivePDO3Parameter = 0x011402,
        ON_ReceivePDO4Parameter = 0x011403,
        ON_ReceivePDO1Type = 0x021400,
        ON_ReceivePDO2Type = 0x021401,
        ON_ReceivePDO3Type = 0x021402,
        ON_ReceivePDO4Type = 0x021403,
        ON_ReceivePDO1Mapping = 0x001600,
        ON_ReceivePDO2Mapping = 0x001601,
        ON_ReceivePDO3Mapping = 0x001602,
        ON_ReceivePDO4Mapping = 0x001603,
        ON_TransmitPDO1Parameter = 0x011800,
        ON_TransmitPDO1Type = 0x021800,
        ON_TransmitPDO1Period = 0x031800,
        ON_TransmitPDO2Parameter = 0x011801,
        ON_TransmitPDO2Type = 0x021801,
        ON_TransmitPDO2Period = 0x031801,
        ON_TransmitPDO3Parameter = 0x011802,
        ON_TransmitPDO3Type = 0x021802,
        ON_TransmitPDO3Period = 0x031802,
        ON_TransmitPDO4Parameter = 0x011803,
        ON_TransmitPDO4Type = 0x021803,
        ON_TransmitPDO4Period = 0x031803,
        ON_TransmitPDO1Mapping = 0x001A00,
        ON_TransmitPDO2Mapping = 0x001A01,
        ON_TransmitPDO3Mapping = 0x001A02,
        ON_TransmitPDO4Mapping = 0x001A03,
        ON_RS232Baudrate = 0x002002,
        ON_CANBitrate = 0x002001,
        ON_EncoderCounter = 0x002020,
        ON_EncoderIndexPulse = 0x002021,
        ON_CurrentActualValueAveraged = 0x002027,
        ON_VelocityActualValueAveraged = 0x002028,
        ON_CurrentModeSetCurrent = 0x002030,//u16
        ON_PositionModeSetPosition = 0x002062,
        On_HomingCurrentThreshold = 0x002080,
        On_HomePosition = 0x002081, // i32
        ON_EncoderPulseNumber = 0x012210,
        ON_PositionSensorType = 0x022210,
        //ON_InternalUsed = 0x032210,
        ON_PositionSensorPolarity = 0x042210,
        ON_Controlword = 0x006040,
        ON_Statusword = 0x006041,
        ON_SetOperationMode = 0x006060,
        ON_ReadPosition = 0x006064,
        ON_VelocitySensorActualValue = 0x006069,
        ON_VelocityActualValue = 0x00606C,
        ON_ReadCurrent = 0x006078,
        ON_HomeOffset = 0x00607C,
        ON_SoftwarePositionLimitMin = 0x01607D,
        ON_SoftwarePositionLimitMax = 0x02607D,
        ON_HomingMethod = 0x006098, // i8
        ON_HomingSpeedForSwitchSearch = 0x016099, // u32
        ON_HomingSpeedForZeroSearch = 0x026099, // u32
        ON_HomingAcceleration = 0x00609A, // u32
        ON_MotorType = 0x006402,
        ON_MotorContinuousCurrentLimit = 0x016410,
        ON_MotorOutputCurrentLimit = 0x026410,
        ON_MotorPolePairNumber = 0x036410,
        ON_MotorMaxSpeedCurrentMode = 0x046410,
        ON_MotorThermalTimeWinding = 0x056410,
    };

    struct ErrorRegister
    {
        uint8_t generic_error : 1;
        uint8_t current_error : 1;
        uint8_t voltage_error : 1;
        uint8_t temperature_error : 1;
        uint8_t communication_error : 1;
        uint8_t device_profile : 1;
        uint8_t reserved : 1; // always 0
        uint8_t motion_error : 1;
    };

    struct Controlword
    {
        uint16_t switch_on : 1;
        uint16_t enable_voltage : 1;
        uint16_t quick_stop : 1;
        uint16_t enable_operation : 1;
        uint16_t setpoint_homingstart : 1;
        uint16_t changeset : 1;
        uint16_t absrel : 1;
        uint16_t fault_reset : 1;
        uint16_t halt : 1;
        uint16_t _reserved : 2;
        uint16_t _notused : 5;
    };

    struct Statusword
    {
        uint16_t ready_to_switch : 1;
        uint16_t switched_on : 1;
        uint16_t openration_enable : 1;
        uint16_t fault : 1;
        uint16_t voltage_enabled : 1; // power stage on
        uint16_t quick_stop : 1;
        uint16_t switch_on_disable : 1;
        uint16_t _notused : 1;
        uint16_t offset_current_measured : 1;
        uint16_t remote : 1; // NMT slave state operational
        uint16_t target_reached : 1;
        uint16_t internal_limit_active : 1;
        uint16_t setpoint_speed_homing : 1;
        uint16_t following_homingerr : 1;
        uint16_t refresh_power_cycle : 1;
        uint16_t position_referenced_home : 1;
    };

    struct MiscellaneousConfiguration
    {
        uint16_t disable_position_supervision : 1;
        uint16_t reserved0 : 1;
        uint16_t motor_resistance_measure : 1;
        uint16_t motor_speed : 1;
        uint16_t reserved1 : 12;
    };      

    enum TransferType {
        TT_Normal=0,
        TT_Expedited=1
    };

    enum OperationMode
    {
        OM_None=0,
        OM_ProfilePosition = 1,
        OM_ProfileVelocity = 3,
        OM_Homing = 6,
        // raw operation commands
        OM_Position=0xff,
        OM_Velcoity=0xfe,
        OM_Current=0xfd,
        OM_Diagnostics=0xfc,
        OM_MasterEncoder=0xfb,
        OM_StepDirection=0xfa,
    };

    enum HomingMethod
    {
        HM_ActualPosition = 35,
        HM_IndexPositiveSpeed = 34,
        HM_IndexNegaiveSpeed = 33,
        HM_HomeSwitchNegativeSpeed = 27,
        HM_HomeSwitchPositiveSpeed = 23,
        HM_PositiveLimitSwitch = 18,
        HM_NegativeLimitSwitch = 17,
        HM_HomeSwitchNegativeSpeedIndex = 11,
        HM_HomeSwitchPositiveSpeedIndex = 7,
        HM_NegativeLimitSwitchIndex = 1,
        HM_NoHoming = 0,
        HM_CurrentThresholdPositiveSpeedIndex = -1,
        HM_CurrentThresholdNegativeSpeedIndex = -2,
        HM_CurrentThresholdPositiveSpeed = -3,
        HM_CurrentThresholdNegativeSpeed = 4
    };

    enum EPOSState
    {
        EPOS_Initialization=0,
        EPOS_PreOperational=1,
        EPOS_Operational=2,
        EPOS_Stopped=3
    };

    /// represents only one EPOS device
    class EPOSMotor
    {
    public:
        EPOSMotor(boost::shared_ptr<EPOSController> pcontroller,const XMLMotorData::MOTORPARAMETERS& motorparameters, int nodeid) : _pcontroller(pcontroller), _nodeid(nodeid), _motorparameters(motorparameters)
        {
            _fposition = 0; _fvelocity = 0; _factualtorque = 0; _fcommandtorque = 0;
            _state = EPOS_PreOperational;
            _mode = OM_None;
            
            WriteObjectName(ON_TransmitPDO1Parameter,0x40000180+_nodeid);
            WriteObjectName(ON_TransmitPDO1Type,(uint8_t)1); // set to SYNC
            WriteObjectName(ON_TransmitPDO1Mapping,(uint8_t)0); // reset PDO
            WriteObjectName((ObjectName)(ON_TransmitPDO1Mapping+0x010000),0x60640020); // actual position value
            WriteObjectName((ObjectName)(ON_TransmitPDO1Mapping+0x020000),0x20280020); // actual averaged velocity
            WriteObjectName(ON_TransmitPDO1Mapping,(uint8_t)2); // number of objects
            WriteObjectName(ON_TransmitPDO1Period,(uint16_t)1);

            WriteObjectName(ON_TransmitPDO2Parameter,0x40000280+_nodeid);
            WriteObjectName(ON_TransmitPDO2Type,(uint8_t)1); // set to SYNC
            WriteObjectName(ON_TransmitPDO2Mapping,(uint8_t)0); // reset PDO
            WriteObjectName((ObjectName)(ON_TransmitPDO2Mapping+0x010000),0x20270010); // actual averaged current
            WriteObjectName((ObjectName)(ON_TransmitPDO2Mapping+0x020000),0x60410010); // status word
            WriteObjectName(ON_TransmitPDO2Mapping,(uint8_t)2); // number of objects
            WriteObjectName(ON_TransmitPDO2Period,(uint16_t)1);

            WriteObjectName(ON_TransmitPDO3Parameter,0xC0000380+_nodeid);
            WriteObjectName(ON_TransmitPDO4Parameter,0xC0000480+_nodeid);
            
            WriteObjectName(ON_ReceivePDO1Parameter,0x00000200+_nodeid);
            WriteObjectName(ON_ReceivePDO1Type,(uint8_t)1); // set to SYNC
            WriteObjectName(ON_ReceivePDO1Mapping,(uint8_t)0); // reset PDO
            WriteObjectName((ObjectName)(ON_ReceivePDO1Mapping+0x010000),0x20300010); // current mA
            WriteObjectName((ObjectName)(ON_ReceivePDO1Mapping+0x020000),0x60400010); // control word
            WriteObjectName(ON_ReceivePDO1Mapping,(uint8_t)2);

            Controlword control; memset(&control,0,sizeof(control));
            control.fault_reset = 1;
            control.enable_voltage = 1;
            control.quick_stop = 1;
            WriteObjectName(ON_Controlword,control);

//            MiscellaneousConfiguration cfg;
//            memset(&cfg,0,sizeof(cfg));
//            //cfg.motor_speed = 1; // measures velocity by encoder pulse time
//            WriteObjectName(ON_MiscellaneousConfiguration,*(uint16_t*)&cfg);

            if( ReadObjectName(ON_PositionSensorType) != 1 )
                ROS_WARN("encoder is not 3 channel");
            
            int nEncoderTicksPerRevolution = ReadObjectName(ON_EncoderPulseNumber)*4;
            _fencoderratio = 1.0f / (motorparameters.transmissionratio * nEncoderTicksPerRevolution);
            _fvelocityratio = 1.0f / (60.0f * motorparameters.transmissionratio);
            _homeposition = ReadObjectName(On_HomePosition);

            WriteObjectName(ON_MotorMaxSpeedCurrentMode,(uint16_t)_motorparameters.maxrpmspeed);
            
            _syncid = ReadObjectName(ON_SYNC);
            ROS_DEBUG("node%d: sync id: 0x%x, homepos: 0x%x, motor type: %d",_nodeid,_syncid,_homeposition,ReadObjectName(ON_MotorType));
            ROS_DEBUG("node%d: motor continuous current limit: %dmA",_nodeid,ReadObjectName(ON_MotorContinuousCurrentLimit));
            ROS_DEBUG("node%d: motor output current limit: %dmA",_nodeid,ReadObjectName(ON_MotorOutputCurrentLimit));
            ROS_DEBUG("node%d: motor pole pair number: %d",_nodeid,ReadObjectName(ON_MotorPolePairNumber));
            ROS_DEBUG("node%d: motor max speed current %d rpm",_nodeid,ReadObjectName(ON_MotorMaxSpeedCurrentMode));
            ROS_DEBUG("node%d: motor thermal time winding: %d",_nodeid,ReadObjectName(ON_MotorThermalTimeWinding));
        }

        int ReadObjectName(ObjectName name)
        {
            if( !ReadSDO(name) )
                throw controller_exception("failed to read");

            uint32_t starttime = timeGetTime();
            CMSG msg;
            int response = 0;
            while(timeGetTime()-starttime<400) {
                if( _pcontroller->_can.CanGetMsg(msg) ) {
                    if( ReadSDOResponse(msg,name,response) )
                        return response;
                    else {
                        ROS_WARN("ReadObjectName %x ignoring can id msg 0x%x: 0x%.8x%.8x",name,msg.id,*(uint32_t*)&msg.data[4],*(uint32_t*)&msg.data[0]);
                    }
                }
            }

            throw controller_exception("timed out");
            return response;
        }

        template <typename T>
        void WriteObjectName(ObjectName name, T value)
        {
            if( !WriteSDO(name,value) )
                throw controller_exception("failed to read");

            uint32_t starttime = timeGetTime();
            CMSG msg;
            while(timeGetTime()-starttime<400) {
                if( _pcontroller->_can.CanGetMsg(msg) ) {
                    if( WriteSDOResponse(msg,name) )
                        return;
                    else
                        ROS_WARN("WriteObjectName %x ignoring can id msg 0x%x: 0x%.8x%.8x",name, msg.id,*(uint32_t*)&msg.data[4],*(uint32_t*)&msg.data[0]);
                }
            }

            throw controller_exception("timed out");
        }

        bool ReadSDO(ObjectName name)
        {
            uint8_t data[8];
            data[0] = 0x40;
            *(int*)&data[1] = name;
            if( !_pcontroller->_can.CanSendRaw(0x600+_nodeid,4,0,data) ) {
                ROS_WARN("ReadSDO: node %d, failed to send can, status: %x",_nodeid,*(uint16_t*)&_status);
                return false;
            }
            return true;
        }

        bool ReadSDOResponse(CMSG& msg,ObjectName name,int& response)
        {
            if( msg.id != 0x580+_nodeid )
                return false;

            if( (msg.data[0] & 0xe0) != 0x40 ) {
                if( (msg.data[0] & 0xe0) == 0xc0 ) {
                    ErrorCode ec = *(ErrorCode*)&msg.data[4];
                    ROS_ERROR("abort error(0x%x): %s",ec,_pcontroller->_mapErrorCodes[ec].c_str());
                    return false;
                }
                
                throw controller_exception("unknown tag");
            }

            if( ((*(int*)&msg.data[1])&0xffffff) != name )
                return false;

            int numvalid = 4;
            TransferType type = (TransferType)((msg.data[0]>>1)&1);
            if( (msg.data[0] & 1) && type == TT_Expedited )
                numvalid = 4-((msg.data[0]>>2)&3);
            response = (*(int*)&msg.data[4])&(0xffffffff>>(4-numvalid));
            //ROS_DEBUG("read response %.6x: %d",name,response);
            return true;
        }

        template <typename T>
        bool WriteSDO(ObjectName name,T value)
        {
            assert(sizeof(T)<=4);
            uint8_t data[8];
            data[0] = 0x23|((4-sizeof(T))<<2);
            *(int*)&data[1] = name;
            *(T*)&data[4] = value;
            if( !_pcontroller->_can.CanSendRaw(0x600+_nodeid,4+sizeof(T),0,data) ) {
                ROS_WARN("WriteSDO: node %d, failed to send can, status: %x",_nodeid,*(uint16_t*)&_status);
                return false;
            }
            return true;
        }

        bool WriteSDOResponse(CMSG& msg,ObjectName name)
        {
            if( msg.id != 0x580+_nodeid )
                return false;

            if( (msg.data[0] & 0xe0) != 0x60 ) {
                if( (msg.data[0] & 0xe0) == 0x80 ) {
                    ErrorCode ec = *(ErrorCode*)&msg.data[4];
                    ROS_ERROR("abort error(0x%x): %s",ec,_pcontroller->_mapErrorCodes[ec].c_str());
                    return false;
                }
                
                throw controller_exception("unknown tag");
            }
            
            return ((*(int*)&msg.data[1])&0xffffff) == name;
        }

        /// returns true if message was successfully processed
        bool ProcessMessage(const CMSG& msg)
        {
            if( msg.id == 0x180+_nodeid ) {
                if( msg.len != 8 )
                    ROS_ERROR("0x%x message not enough length",msg.id);
                else {
                    _fposition = (*(int*)&msg.data[0]-_homeposition)*_fencoderratio;
                    _fvelocity = *(int*)&msg.data[4]*_fvelocityratio;
                }

                return true;
            }
            else if( msg.id == 0x280+_nodeid ) {
                if( msg.len != 4 )
                    ROS_ERROR("0x%x message not enough length",msg.id);
                else {
                    _factualtorque = (float)*(int16_t*)&msg.data[0]/(1000.0f*_motorparameters.torquetocurrent);
                    _status = *(Statusword*)&msg.data[2];
                }
                return true;
            }
            return false;
        }

        void SetOperationMode(OperationMode mode)
        {
            if( mode == _mode )
                return;

            if( _state != EPOS_PreOperational ) {
                ROS_INFO("setting node %d to pre-operational",_nodeid);
                uint8_t data[8]={0};
                data[0] = 0x80; // pre-operational
                data[1] = _nodeid;
                if( !_pcontroller->_can.CanSendRaw(0, 2, 0, data) ) {
                    ROS_WARN("SetOperationMode: node %d, failed to send can, status: %x",_nodeid,*(uint16_t*)&_status);
                }
                
                _state = EPOS_PreOperational;
                usleep(100000);
            }

            if( mode != OM_None ) {
                ROS_INFO("setting node %d to mode %d",_nodeid,mode);
                Controlword control; memset(&control,0,sizeof(control));
                control.enable_voltage = 1;
                control.quick_stop = 1;
                WriteObjectName(ON_Controlword,control);
                usleep(10000);

                // set current mode and enable the motor
                WriteObjectName(ON_SetOperationMode,(uint8_t)mode);

                memset(&control,0,sizeof(control));
                control.switch_on = 1;
                control.enable_voltage = 1;
                control.quick_stop = 1;
                control.enable_operation = 1;
                WriteObjectName(ON_Controlword,control);
                usleep(10000);

                WriteObjectName(ON_CurrentModeSetCurrent,(int16_t)0);

                if( _state != EPOS_Operational ) {
                    ROS_INFO("setting node %d to operational",_nodeid);
                    uint8_t data[8]={0};
                    data[0] = 0x01; // operational
                    data[1] = _nodeid;
                    if( !_pcontroller->_can.CanSendRaw(0, 2, 0, data) ) {
                        ROS_WARN("SetOperationMode: node %d, failed to send can operational, status: %x",_nodeid,*(uint16_t*)&_status);
                    }
                    else
                        _state = EPOS_Operational;
                    usleep(100000);
                }
            }

            _mode = mode;
        }

        void SetHomePosition(dReal fmotorpos)
        {
            _homeposition += (int)((_fposition-fmotorpos)/_fencoderratio);
            _fposition = fmotorpos;
            ROS_INFO("motor %d calibrated: 0x%x",_nodeid,_homeposition);
        }

        bool SendControl()
        {
            Controlword control; memset(&control,0,sizeof(control));
            control.switch_on = 1;
            control.enable_voltage = 1;
            control.quick_stop = 1;
            control.enable_operation = 1;

            uint16_t data[4];
            data[0] = (int16_t)(_fcommandtorque*_motorparameters.torquetocurrent*1000.0f);
            data[1] = *(uint8_t*)&control;
            if( !_pcontroller->_can.CanSendRaw(0x200+_nodeid,4,0,data) ) {
                ROS_WARN("SendControl: node %d, failed to send can, status: %x",_nodeid,*(uint16_t*)&_status);
                return false;
            }
            return true;
        }

        dReal GetPosition() { return _fposition; }
        dReal GetVelocity() { return _fvelocity; }
        dReal GetActualTorque() { return _factualtorque; }
        int GetSyncId() const { return _syncid; }
        Statusword GetStatus() const { return _status; }
        uint16_t GetStatusValue() const { return *(uint16_t*)&_status; }
        EPOSState GetState() const { return _state; }

        const XMLMotorData::MOTORPARAMETERS& GetParameters() const { return _motorparameters; }

        dReal _fcommandtorque;
        int _homeposition;

    private:
        boost::shared_ptr<EPOSController> _pcontroller;
        int _nodeid, _syncid;
        XMLMotorData::MOTORPARAMETERS _motorparameters;
        dReal _fencoderratio, _fvelocityratio;
        Statusword _status;
        EPOSState _state;
        OperationMode _mode;
        
        /// runtime parameters
        //@{
        dReal _fposition, _fvelocity, _factualtorque;
        //@}
    };

    /// parameters and state for one joint of the robot
    class Joint
    {
    public:
        Joint(const XMLMotorData::JOINTPARAMETERS& jointparameters, int jointindex) : _jointparameters(jointparameters), _jointindex(jointindex) {
            _ftargetpos = _ftargetvel = _ftargetaccel = _fcommandtorque = 0;
            _IntegralTerm = 0;
            _bControlPosition = false;
            _popenravejoint = NULL;
        }
        
        int GetJointIndex() const { return _jointindex; }
        const XMLMotorData::JOINTPARAMETERS& GetParameters() const { return _jointparameters; }
        bool IsDiffDrive() { return !!_motors[1]; }

        dReal GetPosition()
        {
            return (!_motors[0]?0:_motors[0]->GetPosition()*_jointparameters.motortransmission[0])
                + (!_motors[1]?0:_motors[1]->GetPosition()*_jointparameters.motortransmission[1]);
        }
        dReal GetVelocity()
        {
            return (!_motors[0]?0:_motors[0]->GetVelocity()*_jointparameters.motortransmission[0])
                + (!_motors[1]?0:_motors[1]->GetVelocity()*_jointparameters.motortransmission[1]);
        }
        dReal GetActualTorque()
        {
            return (!_motors[0]?0:_motors[0]->GetActualTorque()*_jointparameters.motortransmission[0])
                    + (!_motors[1]?0:_motors[1]->GetActualTorque()*_jointparameters.motortransmission[1]);
        }
        dReal GetCommandedTorque()
        {
            return _fcommandtorque;
        }

        void SetPosition(dReal ftargetpos, dReal ftargetvel=0, dReal ftargetaccel=0)
        {
            _ftargetpos = ftargetpos;
            _ftargetvel = ftargetvel;
            _ftargetaccel = ftargetaccel;
            _IntegralTerm = 0;
            _bControlPosition = true;
            _fcommandtorque = 0;
        }

        void SetIdle()
        {
            _bControlPosition = false;
            _IntegralTerm = 0;
            _fcommandtorque = 0;
        }

        void ProcessCommands(dReal fElapsedTime)
        {
            _fcommandtorque = 0;
            if( _bControlPosition ) {
                dReal poserror = _ftargetpos-GetPosition();
                dReal velerror = _ftargetvel-GetVelocity();
                _IntegralTerm += fElapsedTime*poserror;
                dReal integral = _IntegralTerm*_jointparameters.igain;
                if( integral < -_jointparameters.maxintegral)
                    integral = -_jointparameters.maxintegral;
                else if( integral > _jointparameters.maxintegral)
                    integral = _jointparameters.maxintegral;
                _fcommandtorque += _jointparameters.pgain*poserror + _jointparameters.dgain*velerror + integral;
                //ROS_INFO("cmdtorque%d: %f, pos: %f, targ: %f",_jointindex,_fcommandtorque,GetPosition(),_ftargetpos);
            }
        }

        boost::shared_ptr<EPOSMotor> _motors[2];
        KinBody::Joint* _popenravejoint;
        dReal _ftargetpos, _ftargetvel, _ftargetaccel, _fcommandtorque;
        dReal _IntegralTerm;
    private:
        XMLMotorData::JOINTPARAMETERS _jointparameters;
        int _jointindex;
        bool _bControlPosition;
    };

public:
    EPOSController(const int canport, const string& robotfile, const string& manipname, float fMaxVelMult) : OpenRAVEController(robotfile, manipname, vector<string>(), fMaxVelMult), _canport(canport), _bConnected(false), _controltime(0.004) {
        _fElapsedTime = 0;
        _InitStaticData();
    }

    virtual ~EPOSController() {
        _bShutdown = true;
        _disconnect();
        _threadControl.join();
        OpenRAVEController::shutdown();
    }

    virtual void init()
    {
        _fElapsedTime = 0;
        OpenRAVEController::init();
//        log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
//        logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
//        ros::console::notifyLoggerLevelsChanged();
        _threadControl = boost::thread(boost::bind(&EPOSController::_ControlThread,this));
    }

    virtual void shutdown()
    {
        _bShutdown = true;
        _disconnect();
        _threadControl.join();
        OpenRAVEController::shutdown();
    }

protected:
    virtual void _connect()
    {
        boost::mutex::scoped_lock lock(_mutexControl);

        if( !_probot )
            throw controller_exception("failed to open openrave robot");

        EnvironmentMutex::scoped_lock lock(_probot->GetEnv()->GetMutex());
        XMLMotorData* pmotordata = (XMLMotorData*)_probot->GetReadableInterface("motordata");
        if( pmotordata == NULL )
            throw controller_exception("no motor data specified in robot, cannot get motor models");

        if( !_can.CanInit(_canport,1000) )
            throw controller_exception("failed to create CAN");

        uint8_t data[8]={0};
//        {
//            // resets software limits also, which is not desired. so far, have not found a need for "reset"
//            data[0] = 0x81; // reset
//            _can.CanSendRaw(0, 2, 0, data);
//            usleep(50000);
//        }
        data[0] = 0x82; // reset communication
        _can.CanSendRaw(0, 2, 0, data);
        usleep(50000);
        data[0] = 0x80; // preoperation
        _can.CanSendRaw(0, 2, 0, data);
        usleep(400000);

        _mapeposmotors.clear();
        _mapeposmotorsNodeId.clear();
        FOREACH(itmotor, pmotordata->_motors) {
            try {
                if( _mapeposmotors.find(itmotor->first) != _mapeposmotors.end() ) {
                    ROS_WARN("two motors have same name!, skipping...");
                    continue;
                }
                
                int nodeid = boost::lexical_cast<int>(itmotor->first);
                if( _mapeposmotorsNodeId.find(nodeid) != _mapeposmotorsNodeId.end() ) {
                    ROS_WARN("two motors have same node id!, skipping...");
                    continue;
                }

                _mapeposmotorsNodeId[nodeid] = _mapeposmotors[itmotor->first] = boost::shared_ptr<EPOSMotor>(new EPOSMotor(as<EPOSController>(),itmotor->second,nodeid));
                ROS_DEBUG("setting robot joint %s to CAN Node Id %d",itmotor->first.c_str(),nodeid);
            }
            catch(controller_exception& err) {
                ROS_WARN("failed to create motor %s, exception: %s",itmotor->first.c_str(),err.what());
            }
        }

        _vjoints.clear();
        int jointindex = 0;
        FOREACH(itname, _vActiveJointNames) {
            map<string,XMLMotorData::JOINTPARAMETERS>::iterator it = pmotordata->_joints.find(*itname);
            if( it != pmotordata->_joints.end() ) {
                boost::shared_ptr<Joint> joint(new Joint(it->second,jointindex));
                for(int i = 0; i < 2; ++i) {
                    map<string, boost::shared_ptr<EPOSMotor> >::iterator it = _mapeposmotors.find(joint->GetParameters().motornames[i]);
                    if( it != _mapeposmotors.end() )
                        joint->_motors[i] = it->second;
                }
                joint->_popenravejoint = _probot->GetJoints()[_probot->GetActiveJointIndices()[jointindex]];
                _vjoints.push_back(joint);
            }
            else
                ROS_WARN("failed to find joint %s motor definition, disabling",itname->c_str());
            jointindex++;
        }

        _bConnected = true;
        ROS_INFO("connected to robot");
    }

    virtual void _disconnect()
    {
        if( !_bConnected )
            return;

        ROS_INFO("disconnecting robot");
        boost::mutex::scoped_lock lock(_mutexControl);
        _bConnected = false;

        // flush the CAN buffer of any messages
        CMSG msg;
        ros::Time starttime = ros::Time::now();
        while( (ros::Time::now()-starttime) < ros::Duration(0.1) )
            _can.CanGetMsg(msg);

        Controlword control; memset(&control,0,sizeof(control));
        control.switch_on = 1;
        control.enable_voltage = 0;
        control.quick_stop = 1;
        control.enable_operation = 0;
        FOREACH(itmotor, _mapeposmotors) {
            itmotor->second->WriteObjectName(ON_Controlword,control);
        }

        _can.CanDestroy();
    }

    static bool HomingCompare(const boost::shared_ptr<Joint>& m1, const boost::shared_ptr<Joint>& m2)
    {
        return m1->GetParameters().homingorder < m2->GetParameters().homingorder;
    }

    virtual bool _NeedCalibration()
    {
        boost::mutex::scoped_lock lock(_mutexControl);
        FOREACH(itmotor, _mapeposmotors) {
            if( itmotor->second->ReadObjectName(ON_SoftwarePositionLimitMin) == (int)0x80000000 ||
                itmotor->second->ReadObjectName(ON_SoftwarePositionLimitMax) == 0x7fffffff ) {
                return true;
            }
        }

        return false;
    }

    virtual void _calibrate()
    {
        vector<dReal> lower, upper;
        _probot->GetActiveDOFLimits(lower, upper);
        if( !_NeedCalibration() )
            return;
        
            // check if calibration necessary
        ROS_INFO("Joint encoders need to be calibrated.\nTo skip calibration type 'q'.\nTo calibrate robot, move robot to the home position and type 'HOME' [Enter]:");
        string cmd;
        cin >> cmd;
        if( _bShutdown )
            return;

        if( cmd == "q" )
            return;

        if( cmd != "HOME" ) {
            _disconnect();
            exit(1);
        }
        
        {
            boost::mutex::scoped_lock lock(_mutexControl);
            FOREACH(itmotor, _mapeposmotors)
                itmotor->second->SetOperationMode(OM_None);

            FOREACH(itmotor, _mapeposmotors) {
                itmotor->second->WriteObjectName(ON_SoftwarePositionLimitMin,0x80000000);
                itmotor->second->WriteObjectName(ON_SoftwarePositionLimitMax,0x7fffffff);
                itmotor->second->_homeposition = itmotor->second->ReadObjectName(ON_ReadPosition);
            }

            FOREACH(itjoint, _vjoints)
                (*itjoint)->SetIdle();

            FOREACH(itmotor, _mapeposmotors)
                itmotor->second->SetOperationMode(OM_Current);
        }

        vector<dReal> vrobotjointvalues(_probot->GetActiveDOF(),0);
        vector<boost::shared_ptr<Joint> > vjoints = _vjoints;
        sort(vjoints.begin(),vjoints.end(),HomingCompare);
        FOREACH(itjoint, vjoints) {
            if( (*itjoint)->GetParameters().homingtorque != 0 ) {
                (*itjoint)->SetIdle();
                (*itjoint)->_ftargetvel = (*itjoint)->_popenravejoint->GetType() == KinBody::Joint::JointSlider ? 0.02f : 0.2f;
                (*itjoint)->_ftargetaccel = (*itjoint)->_ftargetvel;
                _CalibrationLoop(*itjoint);
                if( _bShutdown )
                    return;

                dReal fjointpos = (*itjoint)->GetParameters().homingtorque < 0 ? lower[(*itjoint)->GetJointIndex()] : upper[(*itjoint)->GetJointIndex()];
                if( (*itjoint)->IsDiffDrive() ) {
                    // check if other motor is calibrated
                    vector<boost::shared_ptr<Joint> >::iterator itjoint2;
                    dReal transmission2[2];
                    for(itjoint2 = vjoints.begin(); itjoint2 != itjoint; ++itjoint2) {
                        if( (*itjoint2)->IsDiffDrive() ) {
                            if( (*itjoint)->_motors[0] == (*itjoint2)->_motors[0] && (*itjoint)->_motors[1] == (*itjoint2)->_motors[1] ) {
                                transmission2[0] = (*itjoint2)->GetParameters().motortransmission[0];
                                transmission2[1] = (*itjoint2)->GetParameters().motortransmission[1];
                                break;
                            }
                            else if( (*itjoint)->_motors[0] == (*itjoint2)->_motors[1] && (*itjoint)->_motors[1] == (*itjoint2)->_motors[0] ) {
                                transmission2[0] = (*itjoint2)->GetParameters().motortransmission[1];
                                transmission2[1] = (*itjoint2)->GetParameters().motortransmission[0];
                                break;
                            }
                        }
                    }
                    if( itjoint2 != itjoint ) {
                        dReal positions[2] = {fjointpos, vrobotjointvalues[(*itjoint2)->GetJointIndex()]};
                        dReal A[4] = {(*itjoint)->GetParameters().motortransmission[0],(*itjoint)->GetParameters().motortransmission[1],transmission2[0],transmission2[1]};
                        dReal idet = 1.0f/(A[0]*A[3]-A[1]*A[2]);
                        (*itjoint)->_motors[0]->SetHomePosition(idet*(A[3]*positions[0]-A[1]*positions[1]));
                        (*itjoint)->_motors[1]->SetHomePosition(idet*(-A[2]*positions[0]+A[0]*positions[1]));
                        (*itjoint2)->SetPosition((*itjoint2)->GetPosition());
                    }
                }
                else {
                    (*itjoint)->_motors[0]->SetHomePosition(fjointpos/(*itjoint)->GetParameters().motortransmission[0]);
                }
                vrobotjointvalues[(*itjoint)->GetJointIndex()] = fjointpos;
            }
                
            // set new software position limits
            //(*itmotor)->WriteObjectName(ON_SoftwarePositionLimitMin,0x80000000);
            //(*itmotor)->WriteObjectName(ON_SoftwarePositionLimitMax,0x7fffffff);
            if( (*itjoint)->IsDiffDrive() )
                (*itjoint)->SetPosition((*itjoint)->GetPosition());
            else
                (*itjoint)->SetIdle();
            usleep(5000); // give time for encoders to adjust
        }

        // set devices to pre-operational state to set the new home positions
        FOREACH(itmotor,_mapeposmotors)
            itmotor->second->SetOperationMode(OM_None);
        usleep(100000);
        _ProcessCANMessages(100); // process the left over CAN messages
        FOREACH(itmotor,_mapeposmotors) {
            itmotor->second->WriteObjectName(On_HomePosition,itmotor->second->_homeposition);
            ROS_ASSERT(itmotor->second->ReadObjectName(On_HomePosition)==itmotor->second->_homeposition);
        }

        _probot->SetActiveDOFValues(NULL,&vrobotjointvalues[0],true);
        ROS_INFO("calibration complete");
        FOREACH(itjoint,_vjoints) {
            (*itjoint)->SetIdle();
            (*itjoint)->ProcessCommands(0);
        }
        _SendJointTorques();
    }

    virtual void _ControlThread()
    {
//        _connect();
//        if( _bShutdown )
//            return;
//
//        while(!_bShutdown)
//            _ProcessCANMessages(10);

//        {
//            boost::mutex::scoped_lock lock(_mutexControl);
////            boost::shared_ptr<EPOSMotor> motor = _mapeposmotors["1"];
////            ROS_INFO("limit: %d",motor->ReadObjectName(ON_SoftwarePositionLimitMin));
////            motor->WriteObjectName(ON_SoftwarePositionLimitMin,0);
//            return;
//        }

        ros::Time lasttime = ros::Time::now();
        ros::Time gravitytime = ros::Time::now();
        while(!_bShutdown) {
            try {
                if( !_bConnected ) {
                    _connect();
                    if( !_bConnected ) {
                        usleep(500000);
                        continue;
                    }
                    _calibrate();
                    {
                        boost::mutex::scoped_lock lock(_mutexControl);
                        ROS_DEBUG("setting all epos devices to operational");
                        FOREACH(itmotor, _mapeposmotors)
                            itmotor->second->SetOperationMode(OM_Current);
                    }
                }

                ros::Time starttime = ros::Time::now();

                {
                    boost::mutex::scoped_lock lock(_mutexControl);
                    _mstate.header.stamp = starttime;
                    // send sync for next encoders
                    FOREACH(itjoint, _vjoints) {
                        //ss << (*itjoint)->GetPosition() << " ";
                        _mstate.position[(*itjoint)->GetJointIndex()] = (*itjoint)->GetPosition();
                        _mstate.velocity[(*itjoint)->GetJointIndex()] = (*itjoint)->GetVelocity();
                        _mstate.effort[(*itjoint)->GetJointIndex()] = (*itjoint)->GetActualTorque();
                    }
                    //ROS_INFO(ss.str().c_str());
                }
            
                _pubmstate.publish(_mstate);
                _publishTF();
            
                // process commands
                ros::Time newtime = ros::Time::now();
                _fElapsedTime = (newtime-lasttime).toSec();
                
                {
                    boost::mutex::scoped_lock lock(_mutexCommands);
                    if( _listCommands.size() > 0 ) {
                        if( _listCommands.front()->run() == Status_Finished ) {
                            ROS_DEBUG("command 0x%x finished", _listCommands.front()->getCommandId());
                            _mapFinishedCommands[_listCommands.front()->getCommandId()] = _listCommands.front()->getCommandTime();
                            _listCommands.pop_front();
                            if( _listCommands.size() == 0 )
                                _state = State_Ready;
                            _conditionCommand.notify_all();
                        }
                    }
                    
                    if( _listCommands.size() == 0 && _bConnected ) {
                        boost::mutex::scoped_lock lock(_mutexControl);
                        if( _state == State_Idle ) {
                            // torque on current position
                            if( (newtime-gravitytime).toSec() > 0.1 ) {
                                gravitytime = newtime;
                                FOREACH(itjoint, _vjoints)
                                    (*itjoint)->SetPosition((*itjoint)->GetPosition());
                            }
                            FOREACH(itjoint, _vjoints) {
                                (*itjoint)->ProcessCommands(_fElapsedTime);
                            }
                        }
                        else {
                            // torque on last set position
                            FOREACH(itjoint, _vjoints)
                                (*itjoint)->ProcessCommands(_fElapsedTime);
                        }
                    }
                }

                {
                    boost::mutex::scoped_lock lock(_mutexControl);
                    _SendJointTorques();
                }

                if( ros::Time::now()-starttime < _controltime ) {
                    // process can messages until next control period
                    do {
                        _ProcessCANMessages(10);
                    } while(ros::Time::now()-starttime < _controltime);
                }

                lasttime = newtime;

            }
            catch(controller_exception& err) {
                // reset state to idle and try to reconnect
                if( _bConnected ) {
                    ROS_WARN("resetting EPOS devices and setting robot to idle");
                    _disconnect();
                }
                _clearCommands();
                usleep(200000);
                continue;
            }
        }

        _disconnect();
    }

    virtual void _CalibrationLoop(boost::shared_ptr<Joint> joint)
    {
        ROS_INFO("calibrating joint: %d",joint->GetJointIndex());

        {
            boost::mutex::scoped_lock lock(_mutexControl);
            if( !!joint->_motors[0] )
                joint->_motors[0]->SetOperationMode(OM_Current);
            if( !!joint->_motors[1] )
                joint->_motors[1]->SetOperationMode(OM_Current);
        }
        
        ros::Time lasttime = ros::Time::now();
        dReal ftargetvelocity = 0;
        dReal fThreshTime = 0;

        while(!_bShutdown) {
            ros::Time newtime = ros::Time::now();
            dReal felapsedtime = (newtime-lasttime).toSec();
            ftargetvelocity += felapsedtime*joint->_ftargetaccel;
            ftargetvelocity = min(joint->_ftargetvel,ftargetvelocity);
            
            {
                boost::mutex::scoped_lock lock(_mutexControl);
                FOREACH(itjoint, _vjoints)
                    (*itjoint)->ProcessCommands(felapsedtime);
                
                //ROS_INFO("pos: %f, vel: %f, targ: %f, joint: %d",joint->GetPosition(),joint->GetVelocity(),ftargetvelocity,joint->GetJointIndex());
                dReal fvelerr;
                if( joint->GetParameters().homingtorque < 0 )
                    fvelerr = -ftargetvelocity-joint->GetVelocity();
                else
                    fvelerr = ftargetvelocity-joint->GetVelocity();
                dReal integral = joint->GetParameters().igain*joint->_IntegralTerm;
                if( integral < -joint->GetParameters().maxintegral)
                    integral = -joint->GetParameters().maxintegral;
                else if( integral > joint->GetParameters().maxintegral)
                    integral = joint->GetParameters().maxintegral;
                joint->_fcommandtorque = joint->GetParameters().dgain*fvelerr + integral;
                joint->_IntegralTerm += felapsedtime*fvelerr;
                //ROS_INFO("%f %f",joint->_fcommandtorque,joint->_IntegralTerm);
                _SendJointTorques();
            }

            if( ros::Time::now()-lasttime < _controltime ) {
                // process can messages until next control period
                do {
                    _ProcessCANMessages(10);
                } while(ros::Time::now()-lasttime < _controltime);
            }

            //ROS_INFO("%d: actual torque: %f, commanded: %f, limit %f",joint->GetJointIndex(),joint->GetActualTorque(),joint->_fcommandtorque,joint->GetParameters().homingtorque);
            lasttime = newtime;
            if( fabsf(joint->GetActualTorque()) > fabsf(joint->GetParameters().homingtorque) ) {
                fThreshTime += felapsedtime;
                if( fThreshTime > 0.2f )
                    break;
            }
            else
                fThreshTime = 0;
        }
    }

    virtual void _ProcessCANMessages(int canmaxiter)
    {
        CMSG msg;
        for(int caniter = 0; caniter < canmaxiter; ++caniter) {
            if(!_can.CanGetMsg(msg))
                break;
            int nodeid = msg.id&0x7f;
            if( nodeid >0 && _mapeposmotorsNodeId.find(nodeid) != _mapeposmotorsNodeId.end() ) {
                boost::shared_ptr<EPOSMotor> epos = _mapeposmotorsNodeId[nodeid];
                if( !epos->ProcessMessage(msg) ) {
                    ErrorDeviceCode err = (ErrorDeviceCode)*(uint16_t*)&msg.data[0];
                    if( _mapErrorDeviceCodes.find(err) != _mapErrorDeviceCodes.end() ) {
                        ErrorRegister errreg = *(ErrorRegister*)&msg.data[2];
                        ROS_ERROR("node %d error=0x%x, status=0x%x: %s",nodeid,msg.data[2],epos->GetStatusValue(),_mapErrorDeviceCodes[err].c_str());
                        // messes up queued can messages, so don't call read object name
//                        int numerrors = epos->ReadObjectName(ON_ErrorHistoryNum);
//                        if( numerrors > 0 ) {
//                            for(int i = 1; i <= numerrors; ++i) {
//                                ROS_ERROR("error%d: %.8x",i,epos->ReadObjectName((ObjectName)(ON_ErrorHistoryNum|(i<<16))));
//                            }
//                        }
                    }
                    else {
                        ROS_WARN("unprocessed message: %x: 0x%.8x%.8x",msg.id,*(uint32_t*)&msg.data[4],*(uint32_t*)&msg.data[0]);
                    }
                }
            }
            else {
                ROS_INFO("recv: %x: 0x%.8x%.8x",msg.id,*(uint32_t*)&msg.data[4],*(uint32_t*)&msg.data[0]);
            }
        }
    }

    virtual void _SendJointTorques()
    {
        stringstream ss; ss << "state " << _state << " joint torques: ";
        FOREACH(itjoint, _vjoints) {
            ss << (*itjoint)->_fcommandtorque << " ";
            if( (*itjoint)->IsDiffDrive() ) {
                // find the other joint
                vector<boost::shared_ptr<Joint> >::iterator itjoint2;
                dReal transmission2[2];
                for(itjoint2 = _vjoints.begin(); itjoint2 != _vjoints.end(); ++itjoint2) {
                    if( (*itjoint2)->IsDiffDrive() ) {
                        if( (*itjoint)->_motors[0] == (*itjoint2)->_motors[0] && (*itjoint)->_motors[1] == (*itjoint2)->_motors[1] ) {
                            transmission2[0] = (*itjoint2)->GetParameters().motortransmission[0];
                            transmission2[1] = (*itjoint2)->GetParameters().motortransmission[1];
                            break;
                        }
                        else if( (*itjoint)->_motors[0] == (*itjoint2)->_motors[1] && (*itjoint)->_motors[1] == (*itjoint2)->_motors[0] ) {
                            transmission2[0] = (*itjoint2)->GetParameters().motortransmission[1];
                            transmission2[1] = (*itjoint2)->GetParameters().motortransmission[0];
                            break;
                        }
                    }
                }
                if( itjoint2 == _vjoints.end() )
                    throw controller_exception(str(boost::format("could not find second joint for diff joint %S")%(*itjoint)->_popenravejoint->GetName()));

                dReal torques[2] = {(*itjoint)->_fcommandtorque, (*itjoint2)->_fcommandtorque};
                dReal A[4] = {(*itjoint)->GetParameters().motortransmission[0],(*itjoint)->GetParameters().motortransmission[1],transmission2[0],transmission2[1]};
                dReal idet = 1.0f/(A[0]*A[3]-A[1]*A[2]);
                (*itjoint)->_motors[0]->_fcommandtorque = idet*(A[3]*torques[0]-A[1]*torques[1]);
                (*itjoint)->_motors[1]->_fcommandtorque = idet*(-A[2]*torques[0]+A[0]*torques[1]);
            }
            else if( !!(*itjoint)->_motors[0] ) {
                (*itjoint)->_motors[0]->_fcommandtorque = (*itjoint)->_fcommandtorque / (*itjoint)->GetParameters().motortransmission[0];
            }
        }
        //ROS_INFO(ss.str().c_str());
        
        dReal acttotalcurrent = 0,cmdtotalcurrent = 0;
        FOREACH(itmotor, _mapeposmotors) {
            itmotor->second->SendControl();
            cmdtotalcurrent += fabsf(itmotor->second->_fcommandtorque*itmotor->second->GetParameters().torquetocurrent);
            acttotalcurrent += fabsf(itmotor->second->GetActualTorque()*itmotor->second->GetParameters().torquetocurrent);
        }
        //ROS_INFO("actual current: %f, command current: %f",acttotalcurrent,cmdtotalcurrent);

        // send sync for reading encoder information and sending control
        FOREACH(itmotor, _mapeposmotors) {
            if( !_can.CanSendRaw(itmotor->second->GetSyncId(),0,1,NULL) )
                ROS_WARN("failed to send sync");
            break;
        }
    }

    virtual void _startTrajectoryCommand(boost::shared_ptr<Trajectory> ptraj)
    {
        vector<dReal> vcurrentangles(GetDOF());
        {
            boost::mutex::scoped_lock lock(_mutexControl);
            std::copy(_mstate.position.begin(),_mstate.position.end(),vcurrentangles.begin());

            FOREACH(itjoint,_vjoints)
                (*itjoint)->SetIdle();
        }

        bool bDifferent = false;
        vector<dReal>& vstartangles = ptraj->GetPoints().front().q;
        for(int i = 0; i < GetDOF(); ++i) {
            if( RaveFabs(vcurrentangles[i]-vstartangles[i]) > 0.01f ) {
                bDifferent = true;
                break;
            }
        }

        if( bDifferent ) {
            // have to add the current angles in the beginning, create a new trajectory
            EnvironmentMutex::scoped_lock lock(_probot->GetEnv()->GetMutex());
            boost::shared_ptr<Trajectory> ptesttraj(_penv->CreateTrajectory(GetDOF()));
            ptesttraj->AddPoint(Trajectory::TPOINT(vcurrentangles, 0));
            ptesttraj->AddPoint(Trajectory::TPOINT(vstartangles, 0));
            ptesttraj->CalcTrajTiming(_probot.get(), ptraj->GetInterpMethod(), true, true, _fMaxVelMult);
            dReal fTimeStampOffset = ptesttraj->GetTotalDuration();

            //ROS_DEBUG("adding current angles to beginning of trajectory, offset=%f", fTimeStampOffset);

            FOREACH(itp, ptraj->GetPoints())
                itp->time += fTimeStampOffset;
            Trajectory::TPOINT startpoint(vcurrentangles,0); startpoint.qdot.resize(GetDOF(),0);
            ptraj->GetPoints().insert(ptraj->GetPoints().begin(), startpoint); // add to front
            ptraj->CalcTrajTiming(_probot.get(), ptraj->GetInterpMethod(), false, true, _fMaxVelMult);
        }

        boost::mutex::scoped_lock lock(_mutexControl);
        _state = State_Moving;
    }

    virtual CommandStatus _runTrajectoryCommand(boost::shared_ptr<Trajectory> ptraj, float fCommandTime)
    {
        boost::mutex::scoped_lock lock(_mutexControl);
        vector<dReal> vcurrentangles(GetDOF());
        std::copy(_mstate.position.begin(),_mstate.position.end(),vcurrentangles.begin());

        Trajectory::TPOINT pt;
        if( !ptraj->SampleTrajectory(fCommandTime, pt) ) {
            ROS_WARN("failed to sample trajectory at time %f",fCommandTime);
            return Status_Finished;
        }

        bool bDifferent = false;
        for(int i = 0; i < GetDOF(); ++i) {
            if( RaveFabs(vcurrentangles[i]-pt.q[i]) > 0.01f ) {
                bDifferent = true;
                break;
            }
        }

        // pid control
        FOREACH(itjoint,_vjoints) {
            ROS_DEBUG("%f: joint: %d, pos: %f, target pos: %f, vel: %f, target vel: %f",fCommandTime,(*itjoint)->GetJointIndex(),(*itjoint)->GetPosition(),pt.q[(*itjoint)->GetJointIndex()],(*itjoint)->GetVelocity(),pt.qdot[(*itjoint)->GetJointIndex()]);
            dReal poserror = pt.q[(*itjoint)->GetJointIndex()]-(*itjoint)->GetPosition();
            dReal velerror = -(*itjoint)->GetVelocity();
            if( fCommandTime >= ptraj->GetTotalDuration() )
                velerror += pt.qdot[(*itjoint)->GetJointIndex()];
            (*itjoint)->_fcommandtorque = (*itjoint)->GetParameters().pgain*poserror + (*itjoint)->GetParameters().dgain*velerror + (*itjoint)->GetParameters().igain*(*itjoint)->_IntegralTerm;
            (*itjoint)->_IntegralTerm += _fElapsedTime*poserror;
        }
        
        return (fCommandTime >= ptraj->GetTotalDuration()&&!bDifferent) ? Status_Finished : Status_Running;
    }

    virtual void _finishTrajectoryCommand(boost::shared_ptr<Trajectory> ptraj)
    {
        // freeze the arm to this position
        FOREACH(itjoint,_vjoints)
            (*itjoint)->SetPosition(ptraj->GetPoints().back().q[(*itjoint)->GetJointIndex()]);
    }

    virtual void _startTorqueCommand(vector<dReal> torques)
    {
        _initCheckStall();
        _state = State_Moving;
    }

    virtual CommandStatus _runTorqueCommand(vector<dReal> torques, float fCommandTime)
    {
        CommandStatus cst = Status_Running;
        _state = _checkStall() ? State_Stalled : State_Moving;
        return cst;
    }

    virtual void _finishTorqueCommand()
    {
    }

    virtual void _initCheckStall()
    {
    }

    virtual bool _checkStall()
    {
        return false;
    }

    virtual void _InitEnvironment()
    {
        OpenRAVEController::_InitEnvironment();
        if( !!_penv ) 
            _penv->RegisterXMLReader(PT_Robot, "motordata", EPOSController::CreateMotorDataXMLReader);
    }

    map<ErrorCode,string> _mapErrorCodes;
    map<ErrorDeviceCode,string> _mapErrorDeviceCodes;

    int _canport;
    CAN _can;
    bool _bConnected;
    map<string, boost::shared_ptr<EPOSMotor> > _mapeposmotors; // indexed by motor name
    map<int, boost::shared_ptr<EPOSMotor> > _mapeposmotorsNodeId; // indexed by node id
    vector<boost::shared_ptr<Joint> > _vjoints;
    ros::Duration _controltime;
    dReal _fElapsedTime; // elapsed time since last call to set torque

    boost::thread _threadControl;

protected:
    void _InitStaticData()
    {
        _mapErrorCodes[EC_NoError] = "No error";
        _mapErrorCodes[EC_ToggleError] = "Toggle bit not alternated";
        _mapErrorCodes[EC_SDOTimeOut] = "SDO protocol timed out";
        _mapErrorCodes[EC_ClientServerSpecifierError] = "Client / server command specifier not valid or unknown";
        _mapErrorCodes[EC_OutOfMemoryError] = "Out of memory";
        _mapErrorCodes[EC_AccessError] = "Unsupported access to an object";
        _mapErrorCodes[EC_WriteOnly] = "Read command to a write only object";
        _mapErrorCodes[EC_ReadOnly] = "Write command to a read only object";
        _mapErrorCodes[EC_ObjectDoesNotExist] = "object does not exist";
        _mapErrorCodes[EC_PDOMappingError] = "The object is not mappable to the PDO";
        _mapErrorCodes[EC_PDOLengthError] = "The number and length of the objects to be mapped would exceed PDO length";
        _mapErrorCodes[EC_GeneralParameterError] = "General parameter incompatibility";
        _mapErrorCodes[EC_GeneralInternIncompatibility] = "General internal incompatibility in device";
        _mapErrorCodes[EC_HardwareError] = "Access failed due to an hardware error";
        _mapErrorCodes[EC_ServiceParameterError] = "Data type does not match, length or service parameter does not match";
        _mapErrorCodes[EC_ServiceParameterTooLong] = "Data type does not match, length of service parameter too high";
        _mapErrorCodes[EC_ServiceParameterTooShort] = "Data type does not match, length of service parameter too low";
        _mapErrorCodes[EC_ObjectSubIndexError] = "subindex does not exist";
        _mapErrorCodes[EC_ValueRangeError] = "value range of parameter exeeded";
        _mapErrorCodes[EC_ValueTooHigh] = "Value of parameter written is too high";
        _mapErrorCodes[EC_ValueTooLow] = "value of parameter written is too low";
        _mapErrorCodes[EC_MaxLessMinError] = "maximum value is less than minimum value";
        _mapErrorCodes[EC_GeneralError] = "General error.";
        _mapErrorCodes[EC_TransferStoreError] = "Data cannot be transferred or stored";
        _mapErrorCodes[EC_LocalControlError] = "Data cannot be transferred or stored to application because of local control";
        _mapErrorCodes[EC_WrongDeviceState] = "Data cannot be transferred or stored to application because of the present device state";
        _mapErrorCodes[EC_WrongNMTState] = "wrong NMT state";
        _mapErrorCodes[EC_IllegalCommandError] = "rs232 command illegeal";
        _mapErrorCodes[EC_PasswordError] = "password incorrect";
        _mapErrorCodes[EC_ErrorServiceMode] = "device not in service mode";
        _mapErrorCodes[EC_ErrorCANId] = "Wrong CAN id";

        _mapErrorDeviceCodes[EDC_NoError] = "No error";
        _mapErrorDeviceCodes[EDC_GenericError] = "Unspecific error occurred";
        _mapErrorDeviceCodes[EDC_OverCurrentError] = "Error Cause:\nShort circuit in the motor winding\n"
            "Power supply can not supply enough acceleration current\n"
            "Too high Controller Gains (Velocity control parameter set, Position control parameter set)\n"
            "Profile acceleration and/or Profile deceleration too high\n"
            "Damaged power stage";
        _mapErrorDeviceCodes[EDC_OverVoltageError] = "The power supply voltage is too high";
        _mapErrorDeviceCodes[EDC_UnderVoltage] = "The supply voltage is too low for operation.\n"
            "The power supply can’t supply the acceleration current";
        _mapErrorDeviceCodes[EDC_OverTemperature] = "The temperature at the device power stage is too high (only on EPOS 24/5,EPOS 70/10 and MCD EPOS 60 W)";
        _mapErrorDeviceCodes[EDC_SupplyVoltageLow] = "There is a overload on internal generated 5V supply by the hall sensor connector or encoder connector (only on EPOS 24/5)";
        _mapErrorDeviceCodes[EDC_InternalSoftwareError] = "Internal software error occurred";
        _mapErrorDeviceCodes[EDC_SoftwareParameterError] = "Too high Target position with too low Profile velocity";
        _mapErrorDeviceCodes[EDC_SensorPositionError] = "The detected position from position sensor is no longer valid in case of:\n"
            "- Changed Position Sensor Parameters\n"
            "- Wrong Position Sensor Parameters\n"
            "- Other Errors which influences the absolute position detection (Hall Sensor Error, Encoder Index Error, ...)\n";
        _mapErrorDeviceCodes[EDC_CANOverrunErrorLost] = "One of the CAN mail boxes had a overflow because of too high communication rate";
        _mapErrorDeviceCodes[EDC_CANOverrunError] = "            The execution of the CAN communication had an overrun because of too high communication rate";
        _mapErrorDeviceCodes[EDC_CANPassiveModeError] = "Device changed to CAN passive Mode because:\n"
            "- The CAN baudrate of one CAN node in network is wrong\n"
            "- The CAN network is not connected\n"
            "- The hardware wiring of CAN bus is wrong";
        _mapErrorDeviceCodes[EDC_CANLifeGuardError] = "The CANopen Life Guarding procedure has failed.\n"
            "The Life Guarding is disabled if Guard time = 0";
        _mapErrorDeviceCodes[EDC_CANTransmitCOBIDCollision] = "The device has received a bad transmit PDO request (valid COB-ID without RTR bit set).";
        _mapErrorDeviceCodes[EDC_CANBusOff] = "The CAN Controller has entered CAN bus off state";
        _mapErrorDeviceCodes[EDC_CANRxQueueOverrun] = "One of the CAN receive queues had a overrun because of too high communication rate";
        _mapErrorDeviceCodes[EDC_CANTxQueueOverrun] = "One of the CAN transmit queues had a overrun because of too high communication rate:\n"
            "- too high load on the CAN bus\n"
            "- event triggered PDOs defined with to small inhibit time\n"
            "- too much (synchronous) PDO communication configured for such cycle time";
        _mapErrorDeviceCodes[EDC_CANPDOLengthError] = "The received PDO was not processed due to length error (to short)";
        _mapErrorDeviceCodes[EDC_FollowingError] = "The difference between Position demand value and Position actual value is higher then Maximal following error";
        _mapErrorDeviceCodes[EDC_HallSensorError] = "The motor hall sensors report an impossible signal combination:\n"
            "- Wrong wiring of the hall sensors or the hall sensor supply voltage\n"
            "- Damaged hall sensors of the motor\n"
            "- Big noise on the signal";
        _mapErrorDeviceCodes[EDC_IndexProcessingError] = "The encoder index signal was not found within two turns at start-up because:\n"
            "- Wrong wiring of the encoder cables\n"
            "- Encoder without or with none working index channel\n"
            "- Wrong sensor type (Sensor Configuration)\n"
            "- Too low setting of encoder resolution (Sensor Configuration)\n\n"
            "To many encoder index pulses were detected at unexpected positions because:\n"
            "- Big noise on the encoder signals\n"
            "- Too high input frequency of encoder signals\n";
        _mapErrorDeviceCodes[EDC_EncoderResolutionError] = "The encoder pulses counted between the first two index pulses doesn’t fit to the resolution:\n"
            "- Setting of encoder resolution (Sensor Configuration) is wrong.\n";
        _mapErrorDeviceCodes[EDC_HallsensorNotFound] = "No hall sensor 3 edge found within first motor turn:\n"
            "- Wrong wiring or defect hall sensors\n"
            "- Too low setting of encoder resolution (Sensor Configuration)\n";
        _mapErrorDeviceCodes[EDC_NegativeLimitError] = "- The negative limit switch was or is active\n"
            "- The Configuration of Limit switch function is wrong in Digital Input Functionalities\n";
        _mapErrorDeviceCodes[EDC_PositiveLimitError] = "- The positive limit switch was or is active\n"
            "- The Configuration of Limit switch function is wrong in Digital Input Functionalities\n";
        _mapErrorDeviceCodes[EDC_HallAngleDetectionError] = "The angle difference measured between encoder and hall sensors is too high:\n"
            "- Wrong wiring of Hall sensors or defect Hall sensors\n"
            "- Wrong wiring of encoder or defect encoder\n"
            "- Wrong setting of encoder resolution or pole pairs (Sensor Configuration)\n";
        _mapErrorDeviceCodes[EDC_SoftwarePositionLimit] = "Movement commanded or actual position higher than maximal position limit or lower than minimal position limit (Software position limit)";
        _mapErrorDeviceCodes[EDC_PositionSensorBreach] = "The position sensor supervision has detected a bad working condition\n"
            "- Wrong or broken wiring of encoder\n"
            "- Defect encoder\n"
            "- The regulation parameter are not well tuned (Current control parameter set)\n";
        _mapErrorDeviceCodes[EDC_SystemOverloaded] = "The device has not enough free resources to process the new target value";
    }
};

#endif
