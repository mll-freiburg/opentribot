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

#include <unistd.h>
#include <signal.h>

using namespace std;
using namespace openrave_robot_control;

/// CAN controller for a set of EPOS motor devices
class EPOSController : public OpenRAVEController
{
protected:
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
        EDC_SensorPositionError = 0x6320,
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
        ON_ReadVelocity = 0x002028,
        ON_CurrentModeSetCurrent = 0x002030,
        ON_EncoderPulseNumber = 0x012210,
        ON_PositionSensorType = 0x022210,
        //ON_InternalUsed = 0x032210,
        ON_PositionSensorPolarity = 0x042210,
        ON_Controlword = 0x006040,
        ON_Statusword = 0x006041,
        ON_MotorType = 0x006402,
        ON_MotorContinuousCurrentLimit = 0x016410,
        ON_MotorOutputCurrentLimit = 0x026410,
        ON_MotorPolePairNumber = 0x036410,
        ON_MotorMaxSpeedCurrentMode = 0x046410,
        ON_MotorThermalTimeWinding = 0x056410,
        ON_SetOperationMode = 0x006060,
        ON_ReadPosition = 0x006064,
        ON_ReadCurrent = 0x006078,
        ON_VelocitySensorActualValue = 0x006069,
        ON_VelocityActualValue = 0x00606C,
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

    class CAN
    {
    public:
        NTCAN_HANDLE _canhandle;
        CMSG _canbuffer[16]; // internal receive buffer
        int _iCanCurMsgIndex;
        int _nCanBuffered; 
        int _iCanControl, _iCanMessage, _iCanMessageInfo, _iCanStatus;
        bool _bCanInit;

    CAN() : _bCanInit(false) {
        }
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
                    printf("set baudrate to 1M failed\n");
                    return false;
                }
            }
            else return false;

            // Add all id's ??
            for (int i=0; i<=0x7FF; i++) {
                if (canIdAdd(_canhandle,i)!=NTCAN_SUCCESS) {
                    printf("failed to add %d id\n",i);
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
                if(canTake(_canhandle, _canbuffer, &len)!=NTCAN_SUCCESS ) {
                    printf("canTake error!\n");fflush(stdout);
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
    
            //printf("t msg 0x%x\n, len=%d", id, len);
    
            int size = 1;
            msg.id = id;
            msg.len = len;
            if(rtr)
                msg.len |= NTCAN_RTR;
            memcpy(msg.data,pdata,len);
            bool bsuccess = canSend(_canhandle, &msg, &size) == NTCAN_SUCCESS;
            if( !bsuccess )
                printf("CanSendRaw failed msg = 0x%x\n", id);
            return bsuccess;
        }

        void CanDestroy()
        {
            if( !_bCanInit )
                return;
    
            if (canClose(_canhandle)!=NTCAN_SUCCESS)
                printf("failed to close CAN bus\n");
            _bCanInit = false;
        }
    };

    enum TransferType {
        TT_Normal=0,
        TT_Expedited=1
    };
        
    /// represents only one EPOS device
    class EPOS
    {
    public:
        EPOS(boost::shared_ptr<EPOSController> pcontroller,int nodeid) : _pcontroller(pcontroller), _nodeid(nodeid)
        {
            ROS_INFO("configuring PDO");
            uint8_t data[8]={0};
            data[0] = 0x81; // reset
            _pcontroller->_can.CanSendRaw(0, 2, 0, data);
            usleep(500000);
//            data[0] = 0x82; // reset communication
//            _pcontroller->_can.CanSendRaw(0, 2, 0, data);
//            usleep(50000);
//            data[0] = 0x80; // preoperation
//            _pcontroller->_can.CanSendRaw(0, 2, 0, data);
//            usleep(400000);

            WriteObjectName(ON_TransmitPDO1Parameter,0x40000180+_nodeid);
            WriteObjectName(ON_TransmitPDO1Type,(uint8_t)1); // set to SYNC
            WriteObjectName(ON_TransmitPDO1Mapping,(uint8_t)0); // reset PDO
            WriteObjectName((ObjectName)(ON_TransmitPDO1Mapping+0x010000),0x60640020); // encoder position
            WriteObjectName((ObjectName)(ON_TransmitPDO1Mapping+0x020000),0x60780010); // current actual
            WriteObjectName(ON_TransmitPDO1Mapping,(uint8_t)2); // number of objects
            WriteObjectName(ON_TransmitPDO1Period,(uint16_t)2);

            WriteObjectName(ON_TransmitPDO2Parameter,0x40000280+_nodeid);
            WriteObjectName(ON_TransmitPDO2Type,(uint8_t)1); // set to SYNC
            WriteObjectName(ON_TransmitPDO2Mapping,(uint8_t)0); // reset PDO
            WriteObjectName((ObjectName)(ON_TransmitPDO2Mapping+0x010000),0x60690020); // actual sensor velocity
            WriteObjectName(ON_TransmitPDO2Mapping,(uint8_t)1); // number of objects
            WriteObjectName(ON_TransmitPDO2Period,(uint16_t)2);

            WriteObjectName(ON_TransmitPDO3Parameter,0xC0000380+_nodeid);
            WriteObjectName(ON_TransmitPDO4Parameter,0xC0000480+_nodeid);            
            WriteObjectName(ON_ReceivePDO1Parameter,0x80000200+_nodeid);
            WriteObjectName(ON_ReceivePDO2Parameter,0x80000300+_nodeid);

            Controlword control; memset(&control,0,sizeof(control));
            control.fault_reset = 1;
            control.enable_voltage = 1;
            control.quick_stop = 1;
            WriteObjectName(ON_Controlword,control);

            uint32_t maxspeed = 12500;
            WriteObjectName(ON_MotorMaxSpeedCurrentMode,maxspeed);
            usleep(10000);

            data[0] = 0x01; // operational
            _pcontroller->_can.CanSendRaw(0, 2, 0, data);
            usleep(10000);

//            MiscellaneousConfiguration cfg;
//            memset(&cfg,0,sizeof(cfg));
//            //cfg.motor_speed = 1; // measures velocity by encoder pulse time
//            WriteObjectName(ON_MiscellaneousConfiguration,*(uint16_t*)&cfg);
        }

        int ReadObjectName(ObjectName name)
        {
            if( !ReadSDO(name) )
                throw controller_exception("failed to read");

            uint32_t starttime = timeGetTime();
            CMSG msg;
            int response = 0;
            while(timeGetTime()-starttime<5000) {
                if( _pcontroller->_can.CanGetMsg(msg) ) {
                    if( ReadSDOResponse(msg,name,response) )
                        return response;
                    else
                        ROS_WARN("ignoring can id msg 0x%x: 0x%.8x%.8x",msg.id,*(uint32_t*)&msg.data[4],*(uint32_t*)&msg.data[0]);
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
            while(timeGetTime()-starttime<5000) {
                if( _pcontroller->_can.CanGetMsg(msg) ) {
                    if( WriteSDOResponse(msg,name) )
                        return;
                    else
                        ROS_WARN("ignoring can id msg 0x%x: 0x%.8x%.8x",msg.id,*(uint32_t*)&msg.data[4],*(uint32_t*)&msg.data[0]);
                }
            }

            throw controller_exception("timed out");
        }

        bool ReadSDO(ObjectName name)
        {
            uint8_t data[8];
            data[0] = 0x40;
            *(int*)&data[1] = name;
            return _pcontroller->_can.CanSendRaw(0x600+_nodeid,4,0,data);
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
            ROS_DEBUG("read response %.6x: %d",name,response);
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
            return _pcontroller->_can.CanSendRaw(0x600+_nodeid,4+sizeof(T),0,data);
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

    private:
        boost::shared_ptr<EPOSController> _pcontroller;
        int _nodeid;
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

public:
    EPOSController(const int canport, int nodeid, const string& robotfile, const string& manipname, float fMaxVelMult, float ftargetcurrent, float fmaxdrivetime=1000000.0f) : OpenRAVEController(robotfile, manipname, vector<string>(), fMaxVelMult), _canport(canport), _nodeid(nodeid), _opmode(OM_None), _bConnected(false), _ftargetcurrent(ftargetcurrent),_fmaxdrivetime(fmaxdrivetime) {
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

        _threadControl = boost::thread(boost::bind(&EPOSController::_ControlThread,this));
    }

    virtual ~EPOSController() {
        _bShutdown = true;
        _threadControl.join();
        OpenRAVEController::shutdown();
    }

    virtual void shutdown()
    {
        _bShutdown = true;
        _threadControl.join();
        OpenRAVEController::shutdown();
    }

protected:
    void _ControlThread()
    {
        usleep(10000);
        if( !_can.CanInit(_canport,1000) )
            throw controller_exception("failed to create CAN");
        ROS_INFO("connected to can");

        uint32_t gearratio = 246;
        float fgearmult = 0;

        {
            boost::mutex::scoped_lock lock(_mutexControl);
            _epos.reset(new EPOS(as<EPOSController>(),_nodeid));
            
            if( _epos->ReadObjectName(ON_PositionSensorType) != 1 )
                ROS_WARN("encoder is not 3 channel");
            
            gearratio *= _epos->ReadObjectName(ON_EncoderPulseNumber)*4;
            fgearmult =  6.2831853071795862f / (float)gearratio;

            ROS_INFO("node %d, motor type: %d",_nodeid,_epos->ReadObjectName(ON_MotorType));
            ROS_INFO("motor continuous current limit: %dmA",_epos->ReadObjectName(ON_MotorContinuousCurrentLimit));
            ROS_INFO("motor output current limit: %dmA",_epos->ReadObjectName(ON_MotorOutputCurrentLimit));
            ROS_INFO("motor pole pair number: %d",_epos->ReadObjectName(ON_MotorPolePairNumber));
            ROS_INFO("motor max speed current %d rpm",_epos->ReadObjectName(ON_MotorMaxSpeedCurrentMode));
            ROS_INFO("motor thermal time winding: %d",_epos->ReadObjectName(ON_MotorThermalTimeWinding));

            if( _ftargetcurrent != 0 ) {
                // set current mode
                printf("setting current to %f\n",_ftargetcurrent);
                _opmode = OM_Current;
                _epos->WriteObjectName(ON_SetOperationMode,(uint8_t)OM_Current);
            
                Controlword control; memset(&control,0,sizeof(control));
                control.enable_voltage = 1;
                control.quick_stop = 1;
                _epos->WriteObjectName(ON_Controlword,control);
                usleep(10000);

                control.switch_on = 1;
                control.enable_voltage = 1;
                control.quick_stop = 1;
                control.enable_operation = 1;
                _epos->WriteObjectName(ON_Controlword,control);
                usleep(10000);

                _epos->WriteObjectName(ON_CurrentModeSetCurrent,(int16_t)(1000.0f*_ftargetcurrent));
            }
        }

        CMSG msg;
        list<boost::tuple<float,float,float,float> > listdata;

        int syncid = _epos->ReadObjectName(ON_SYNC);
        uint64_t startime = GetMicroTime();
        ROS_INFO("syncid=%x",syncid);
        //int startencoder = 0;//_epos->ReadObjectName(ON_ReadPosition);

        uint64_t lastsent = 0;

        float fcurvelocity=0;
        int iter = 0;
        while(!_bShutdown) {
            ++iter;

            if( (GetMicroTime()-lastsent) > 500 ) {
                lastsent = GetMicroTime();
                _can.CanSendRaw(syncid,0,1,NULL); // send SYNC
            }
            
            while(_can.CanGetMsg(msg)) {
                if( msg.id == 0x180+_nodeid ) {
                    if( msg.len != 6 )
                        ROS_ERROR("0x181 message not enough length");
                    else {
                        int posencoder = *(int*)&msg.data[0];
                        int16_t readcurrent = *(int16_t*)&msg.data[4];
                        float posrad = posencoder*fgearmult;
                        float elapsedtime = (GetMicroTime()-startime)*1e-6f;
                        //ROS_INFO("%f %d",posrad,readcurrent);
                        listdata.push_back(boost::make_tuple(elapsedtime,posrad,fcurvelocity,0.001f*readcurrent));
                    }
                }
                else if( msg.id == 0x280+_nodeid ) {
                    if( msg.len != 4 )
                        ROS_ERROR("0x280 message not enough length");
                    else {
                        fcurvelocity = *(int*)&msg.data[0]*fgearmult;
                    }
                }
//                else {
//                    ROS_INFO("recv: %x: 0x%.8x%.8x",msg.id,*(uint32_t*)&msg.data[4],*(uint32_t*)&msg.data[0]);
//                }
            }
//
//            int err = _epos->ReadObjectName(ON_ErrorRegister);
//            if( err != 0 )
//                ROS_INFO("error reg: %.2x",err);
//            int numerrors = _epos->ReadObjectName(ON_ErrorHistoryNum);
//            if( numerrors > 0 ) {
//                for(int i = 1; i <= numerrors; ++i) {
//                    ROS_ERROR("error%d: %.8x",i,_epos->ReadObjectName((ObjectName)(ON_ErrorHistoryNum|(i<<16))));
//                }
//            }

            if( _ftargetcurrent != 0 && (GetMicroTime()-startime) > _fmaxdrivetime*1e6) {
                printf("resetting current\n");
                // set current mode
                _opmode = OM_Current;
                _epos->WriteObjectName(ON_SetOperationMode,(uint8_t)OM_Current);
            
                _ftargetcurrent = 0;
                Controlword control; memset(&control,0,sizeof(control));
                control.enable_voltage = 1;
                control.quick_stop = 1;
                _epos->WriteObjectName(ON_Controlword,control);
                usleep(10000);

                control.switch_on = 1;
                control.enable_voltage = 1;
                control.quick_stop = 1;
                control.enable_operation = 1;
                _epos->WriteObjectName(ON_Controlword,control);
                usleep(10000);

                _epos->WriteObjectName(ON_CurrentModeSetCurrent,(int16_t)(1000.0f*_ftargetcurrent));
            }
        }

        FOREACH(it,listdata)
            fprintf(stderr,"%f %f %f %f\n",boost::get<0>(*it),boost::get<1>(*it),boost::get<2>(*it),boost::get<3>(*it));

        // flush the CAN buffer of any messages
        startime = GetMicroTime();
        while( (GetMicroTime()-startime) < 500000 )
            _can.CanGetMsg(msg);

        {
            boost::mutex::scoped_lock lock(_mutexControl);
            Controlword control; memset(&control,0,sizeof(control));
            control.switch_on = 1;
            control.enable_voltage = 0;
            control.quick_stop = 1;
            control.enable_operation = 0;
            _epos->WriteObjectName(ON_Controlword,control);
        }

        _can.CanDestroy();
    }

    map<ErrorCode,string> _mapErrorCodes;

    int _canport;
    CAN _can;
    int _nodeid;
    OperationMode _opmode;
    bool _bConnected;
    boost::shared_ptr<EPOS> _epos;

    boost::thread _threadControl;
    float _ftargetcurrent;
    float _fmaxdrivetime;

    friend class EPOS;
};


void sigint_handler(int);
boost::shared_ptr<openrave_robot_control::OpenRAVEController> s_pcontroller;

int main(int argc, char ** argv)
{
    signal(SIGINT,sigint_handler); // control C
        
    dReal fMaxVelMult = 1;

    // parse the command line options
    string robotname,manipname;
    int canport = 0, nodeid=1;
    float ftargetcurrent = 0, fmaxdrivetime=1e6;
    int i = 1;
    while(i < argc) {
        if( strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0 ) {
            // print the arguments and exit
            printf("robotlinks_filter_node [--can port] [--nodeid id] [--current c] [--maxdrive seconds]\n"
                   "  Start a node to control the EPOS device and publish ROS interfaces.\n"
                   "  Currently the robot file specified has to be in OpenRAVE format\n");
            return 0;
        }
        if( strcmp(argv[i], "--robotfile") == 0 ) {
            robotname = argv[i+1];
            i += 2;
        }
        else if( strcmp(argv[i], "--can") == 0 ) {
            canport = atoi(argv[i+1]);
            i += 2;
        }
        else if( strcmp(argv[i], "--current") == 0 ) {
            ftargetcurrent = atof(argv[i+1]);
            i += 2;
        }
        else if( strcmp(argv[i], "--maxdrive") == 0 ) {
            fmaxdrivetime = atof(argv[i+1]);
            i += 2;
        }
        else if( strcmp(argv[i], "--nodeid") == 0 ) {
            nodeid = atof(argv[i+1]);
            i += 2;
        }
        else
            break;
    }

    ros::init(argc,argv,"testepos", ros::init_options::NoSigintHandler);

    if( !ros::master::check() )
        return 1;
    
    s_pcontroller.reset(new EPOSController(canport, nodeid, robotname, manipname, fMaxVelMult,ftargetcurrent,fmaxdrivetime));
    ros::spin();

    s_pcontroller.reset();
    return 0;
}

void sigint_handler(int)
{
    s_pcontroller->shutdown();
    ros::shutdown();
}
