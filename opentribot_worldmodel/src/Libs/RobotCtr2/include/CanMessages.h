#ifndef _CAN_MESSAGES_H_
#define _CAN_MESSAGES_H_

#include <libpcan.h>
#include <string>
#include <exception>
#include <vector>
#include <pthread.h>

#define ABS10BIT( __x__ ) ((unsigned int) (abs(__x__) > 1023 ? 1023 : abs(__x__)))
#define SGN( __x__ ) (__x__ < 0 ? -1 : 1)
#define BITSGN( __x__ ) (__x__ < 0 ? 1 : 0)


#define CAN_MSG_ID_PING     0x001
#define CAN_MSG_ID_CYCINFO  0x002
#define CAN_MSG_ID_MOTORV   0x003
#define CAN_MSG_ID_MOTORC   0x004
#define CAN_MSG_ID_MOTORO   0x005

#define CAN_MSG_ID_CONFRET  0x00C
#define CAN_MSG_ID_S_CONF   0x00D



#define MOTOR_CTR_MODE_PWM    0
#define MOTOR_CTR_MODE_PID    1

#define CSM_NOTHING 0 
#define CSM_ALL     0xFF

// Conf error codes
#define CONFEC_OK                 0x0000
#define CONFEC_PARAM_OUT_OF_RANGE 0x0001
#define CONFEC_UNKNOWN_CONF_ID    0x0002

#define CONFEC_TIMEOUT            0x0100
#define CONFEC_CANERR             0x0200


  
void write10bitInBuffer( unsigned char* array, int startbyte, int pushleft, unsigned int value);
unsigned short get10bitFromBuffer( const unsigned char* array, int startbyte, int pushright);

void write1bitInBuffer( unsigned char* array, int startbyte, int pushleft, unsigned char value);
unsigned char get1bitFromBuffer( const unsigned char* array, int startbyte, int pushright);

unsigned long get32bitFromBuffer( const unsigned char* array, int startbyte );

void writeUIntInBuffer(unsigned char* array, int startbyte, unsigned int value);

void writeIntInBuffer(      unsigned char* array, int startbyte, unsigned int value);
int  getIntFromBuffer(const unsigned char* array, int startbyte);

namespace RobotCtr2 {
  struct CycleInfoDataMsg {
    CycleInfoDataMsg(const TPCANRdMsg& _rcvMsg);
    CycleInfoDataMsg();
    void setData( const TPCANRdMsg& _rcvMsg );
    std::string sprint();

    long          CanTimeStamp; 
    unsigned char CycleId;
    long          CycleCounter;
    unsigned int  CycleDuration_ms;
    int           CycleIdle_ms;
  };
  
  struct MotorVDataMsg {
    MotorVDataMsg(const TPCANRdMsg& _rcvMsg);
    MotorVDataMsg();
    void setData(const TPCANRdMsg& _rcvMsg);
    std::string sprint();
    
    long          CanTimeStamp;
    unsigned char CycleId;
    int           motorVel0, motorVel1, motorVel2;
    unsigned int  vcc;
    bool          herr0, herr1, herr2;           
  };
  
  struct MotorCDataMsg {
    MotorCDataMsg(const TPCANRdMsg& _rcvMsg);
    MotorCDataMsg();
    void setData(const TPCANRdMsg& _rcvMsg);
    std::string sprint();
    
    long          CanTimeStamp;
    unsigned char CycleId;
    unsigned int  motorC0, motorC1, motorC2;
  };
  
  
  struct MotorODataMsg {
    MotorODataMsg(const TPCANRdMsg& _rcvMsg);
    MotorODataMsg();
    void setData(const TPCANRdMsg& _rcvMsg);
    std::string sprint();
    
    long          CanTimeStamp;
    unsigned char CycleId;
    int           motorO0, motorO1, motorO2;
    
  };

  struct PingDataMsg {
    PingDataMsg(const TPCANRdMsg& _rcvMsg);
    PingDataMsg();
    void setData(const TPCANRdMsg& _rcvMsg);
    std::string sprint();
    
    unsigned char MayerVersion;
    unsigned char LowerVersion;
    char          VersionString[7];
  };

struct ConfRetDataMsg {
  ConfRetDataMsg(const TPCANRdMsg& _rcvMsg);
  ConfRetDataMsg();
  void setData(const TPCANRdMsg& _rcvMsg);
  std::string sprint();
  
  unsigned char ConfId;
  unsigned char confec;
  int Param1;
  int Param2;
  int Param3;
};

struct CanCyclicDataFrame {
  CanCyclicDataFrame();
  void init();
  bool ready();
  int  msgCycId();
  std::string sprint();

  bool add(const TPCANRdMsg& _rcvMsg);
  
  bool             cycInfo_set;
  CycleInfoDataMsg cycInfo;
  bool             vel_set;
  MotorVDataMsg    vel;
  bool             current_set;
  MotorCDataMsg    current;
  bool             output_set;
  MotorODataMsg    output;
  
};


struct ConfMsg {
  ConfMsg();
  ConfMsg(unsigned char ConfId, unsigned char Param1=0,  unsigned int Param2=0,  unsigned int Param3=0,  unsigned int Param4=0);
  void setData(unsigned char ConfId, unsigned char Param1=0,  unsigned int Param2=0,  unsigned int Param3=0,  unsigned int Param4=0);
  TPCANMsg   Msg;
};

}

#endif
