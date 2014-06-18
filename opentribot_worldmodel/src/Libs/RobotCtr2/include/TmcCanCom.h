#ifndef _TMC_CAN_H_
#define _TMC_CAN_H_

#include <string>
#include <exception>
#include <libpcan.h>
#include "CanMessages.h"
#include "MutexedRingBuffer.h"

namespace RobotCtr2 {
  
  enum MotorID {MOTOR1=0, MOTOR2=1, MOTOR3=2};

  class TmcCanException : public std::exception {
  public:
    TmcCanException( const std::string & user_message, 
		     bool includeSysMessage = false)   throw();
    virtual ~TmcCanException() throw() 
      {};
    const char* what() throw() 
      {return message.c_str();};

  protected:
    std::string message;
  };


  /* canbaud: 
   * CAN_BAUD_1M    1 MBit/s, 
   * CAN_BAUD_500K  500 kBit/s, 
   * CAN_BAUD_250K  250 kBit/s,
   * CAN_BAUD_125K  125 kBit/s,
   * CAN_BAUD_100K  100 kBit/s
   * CAN_BAUD_50K    50 kBit/s
   * CAN_BAUD_20K    20 kBit/s
   * CAN_BAUD_10K    10 kBit/s
   * CAN_BAUD_5K      5 kBit/s */
  struct TmcCanCom_Params {
    TmcCanCom_Params();
    
    std::string sprint();

    std::string   DevNodeStr;
    unsigned int  canbaud;
  };

  std::string resolveCanErr(int err);

  class TmcCanCom {
  public:
    TmcCanCom(const TmcCanCom_Params &_params = RobotCtr2::TmcCanCom_Params()) throw(TmcCanException, std::bad_alloc);
    ~TmcCanCom() throw();

     std::string getStatusCyclic();

     bool getLastCyclicFrame(CanCyclicDataFrame& res);
     bool getNextCyclicFrame(CanCyclicDataFrame& res, unsigned long usec_patience = 10000000);
     bool getNewCyclicFrames(std::vector< CanCyclicDataFrame >& res);

     bool ping(std::string * version = 0);

     void         sendMotorVel(int v0, int v1, int v2, unsigned char MotorCtrMode=255, unsigned char kicker1=0, unsigned char kicker2=0);
     unsigned int setCyclicSendMode(unsigned char m);
     unsigned int setPIDParams(MotorID motor, unsigned int gain, unsigned int Tn, unsigned int Tv);
     unsigned int setMaxMotorVel(unsigned int mmv0, unsigned int mmv1, unsigned int mmv2);  // in U/min (>0)

     std::string resolveCONFEC(unsigned int err);
    
     std::string sprintRawRcvMsg(const TPCANRdMsg& rcvMsg);

     unsigned char fw_mayer_version;
     unsigned char fw_lower_version;

  protected:
     TmcCanCom_Params params;

     MutexedRingBuffer< CanCyclicDataFrame >*   cyclicRingBuffer;
     MutexedRingBuffer< PingDataMsg >*          pingMsgBuffer;
     MutexedRingBuffer< ConfRetDataMsg >*       confRetMsgBuffer;

     HANDLE hcyclic;
     //HANDLE hmanual;

     int    lastStatusCyclic;
     //int    lastStatusManual;

     TPCANMsg   reqMsgMan;
     TPCANMsg   sndMsgMan;
     TPCANRdMsg rcvMsgMan;


     pthread_t  theThread;
     bool       runThread;

     void deinit() throw();

     bool stopCyclicMessageColection();
     bool startCyclicMessageColection();

     void getMsgCyclic();
     static void * cyclicMessageCollectionThread(void * tmcCanCom_class);
     
  };
}

#endif
