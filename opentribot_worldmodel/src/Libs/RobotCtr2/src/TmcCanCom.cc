#include "TmcCanCom.h"
#include <iostream>
#include <errno.h>
#include <sstream>

#include <sys/time.h>
#include <fcntl.h>
#include <cstring>
#include <stdlib.h>




//#include <fstream>

//#define DEBUGLEVEL 5

#ifdef DEBUGLEVEL
#define DOUT( __level__ , __x__ ) if ( __level__ <= DEBUGLEVEL) std::cerr << "DEBUG (" << __PRETTY_FUNCTION__ << "): " __x__ << "\n" 
#else
#define DOUT( __level__ , __x__ )
#endif

using namespace std;

RobotCtr2::TmcCanException::TmcCanException( const std::string & user_message, bool includeSysMessage) throw()
  : message(user_message)
{
  if (includeSysMessage) {
    message.append(": ");
    message.append( strerror(errno));
  }
}

RobotCtr2::TmcCanCom_Params::TmcCanCom_Params()  
{
  DevNodeStr = "/dev/pcan32";
  canbaud    = CAN_BAUD_250K;
}

std::string RobotCtr2::TmcCanCom_Params::sprint()
{
  std::stringstream s;

  s << "Can com params:"
    << "\n DevNode: " << DevNodeStr
    << "\n Baud:    ";

  switch (canbaud) {
  case CAN_BAUD_1M:
    s << "CAN_BAUD_1M";
    break;
  case CAN_BAUD_500K:
    s << "CAN_BAUD_500K";
    break;
  case CAN_BAUD_250K:
    s << "CAN_BAUD_250K";
    break;
  case CAN_BAUD_125K:
    s << "CAN_BAUD_125K";
    break;
  case CAN_BAUD_100K:
    s << "CAN_BAUD_100K";
    break;
  case CAN_BAUD_50K:
    s << "CAN_BAUD_50K";
    break;
  case CAN_BAUD_20K:
    s << "CAN_BAUD_20K";
    break;
     case CAN_BAUD_10K:
    s << "CAN_BAUD_10K";
    break;
  case CAN_BAUD_5K:
    s << "CAN_BAUD_5K";
    break;
  default:
    s << "UKNOWN!!";
    break;
  }

  s << "\n";

  return s.str();
}


RobotCtr2::TmcCanCom::TmcCanCom( const RobotCtr2::TmcCanCom_Params &_params ) throw(RobotCtr2::TmcCanException, std::bad_alloc)
{
  params = _params;
  //std::cout << params.sprint();

  hcyclic = NULL;
  //hmanual = NULL;

  runThread = false;

  char chbuf[strlen(params.DevNodeStr.c_str())+1];
  sprintf(chbuf, params.DevNodeStr.c_str(), strlen(params.DevNodeStr.c_str())); 

  hcyclic = LINUX_CAN_Open ( chbuf, O_RDWR );
  //hmanual = LINUX_CAN_Open ( chbuf, O_RDWR );

  if (!hcyclic) {
    hcyclic = 0;
    throw TmcCanException("Can not open can device [" + params.DevNodeStr + "] ", true);
  }

  // clear the can status
  CAN_Status(hcyclic);
  //CAN_Status(hmanual);

  errno = CAN_Init(hcyclic, params.canbaud, CAN_INIT_TYPE_ST);
  //errno = CAN_Init(hmanual, params.canbaud, CAN_INIT_TYPE_ST);

  if (errno) {
    throw TmcCanException("Can not init can device " + resolveCanErr(errno), true);
  }

  cyclicRingBuffer = 0;
  cyclicRingBuffer = new MutexedRingBuffer < CanCyclicDataFrame > (100);
  pingMsgBuffer    = 0;
  pingMsgBuffer    = new MutexedRingBuffer < PingDataMsg > (20);
  confRetMsgBuffer = 0;
  confRetMsgBuffer = new MutexedRingBuffer < ConfRetDataMsg > (20);
  
  if (!startCyclicMessageColection())
    {
      throw TmcCanException("Can't start message collecting thread.", false);
    }

  std::string version;
  if (!ping(&version))
    {
      deinit();
      throw TmcCanException("No answer from TMC, connected?. ", false);
    }

  DOUT(5,"OK. Found device: " << version);
}

RobotCtr2::TmcCanCom::~TmcCanCom() throw()
{
  deinit();
  if (cyclicRingBuffer != 0) delete cyclicRingBuffer;
  if (pingMsgBuffer    != 0) delete pingMsgBuffer;
  if (confRetMsgBuffer != 0) delete confRetMsgBuffer;
  DOUT(5, "Bye. ");
}

void RobotCtr2::TmcCanCom::deinit() throw()
{
  int err;
  stopCyclicMessageColection();

  DOUT(5, "Status cyclic: " << getStatusCyclic());

  if (hcyclic!=0) {
    err = CAN_Close(hcyclic);
    if (err) DOUT(0, "ERROR hcyclic: " << resolveCanErr(err));
    hcyclic = 0;
  }

  DOUT(5, "OK");
}

std::string RobotCtr2::resolveCanErr(int err)
{
  std::string errstring;

  if ( err == 0 ) {
    errstring = "OK";
    return errstring;
  }

  if (err & 0x0001) errstring += "transmit buffer full : ";
  if (err & 0x0002) errstring += "overrun in receive buffer : ";
  if (err & 0x0004) errstring += "bus error (BUSLIGHT), errorcounter limit reached :";
  if (err & 0x0008) errstring += "bus error (BUSHEAVY), errorcounter limit reached :";
  if (err & 0x0010) errstring += "bus error, bus off state entered :";
  if (err & 0x0020) errstring += "receive queue is empty :";
  if (err & 0x0040) errstring += "receive queue overrun :";
  if (err & 0x0080) errstring += "transmit queue full :";
  if (err & 0x0100) errstring += "test of controller register failed :";
  if (err & 0x0200) errstring += "win95/me error:";
  if (err & 0x2000) errstring += "cant create resource :";
  if (err & 0x4000) errstring += "illegal parameter :";
  if (err & 0x8000) errstring += "value out of range :";
  if (err & 0x1C00) errstring += "wrong handle, handle error :";

  return errstring;
}

std::string RobotCtr2::TmcCanCom::resolveCONFEC(unsigned int err)
{
  std::string errstring;

  errstring = "CONF ERROR CODE: ";

  if (err == CONFEC_OK) {
    errstring += "OK";
    return errstring;
  }

  if ( err & CONFEC_PARAM_OUT_OF_RANGE) errstring += "parameter out of range : ";
  if ( err & CONFEC_UNKNOWN_CONF_ID)    errstring += "unknown confid : ";

  if ( err & CONFEC_TIMEOUT)            errstring += "no answer : ";
  if ( err & CONFEC_CANERR)             errstring += "a can error occured :";

  return errstring;
}

std::string RobotCtr2::TmcCanCom::getStatusCyclic()
{
  lastStatusCyclic =  CAN_Status(hcyclic);
  return resolveCanErr(lastStatusCyclic);
}

// std::string TmcCanCom::getStatusManual()
// {
//   lastStatusManual =  CAN_Status(hmanual);
//   return resolveCanErr(lastStatusManual);
// }


std::string RobotCtr2::TmcCanCom::sprintRawRcvMsg(const TPCANRdMsg& _rcvMsg)
{
  std::stringstream res;
  
  res << _rcvMsg.dwTime << "ms ID:" << _rcvMsg.Msg.ID << " TYPE: " << _rcvMsg.Msg.MSGTYPE << " LEN: " << (int) _rcvMsg.Msg.LEN << "\n";

  for (int i=0; i<_rcvMsg.Msg.LEN; i++)
    res << "Byte[" << i << "] : " << _rcvMsg.Msg.DATA[i] << "\n";

  return res.str();
}

bool RobotCtr2::TmcCanCom::startCyclicMessageColection()
{
  if (runThread) return true; // thread already running
  
  runThread = true;
  int ret = pthread_create( &theThread, NULL, cyclicMessageCollectionThread, (void*) this);

  DOUT(5,"Thread created!");
  
  return ( ret == 0);
}

bool RobotCtr2::TmcCanCom::stopCyclicMessageColection()
{
  if (!runThread) return true; // thread not running

  runThread = false;

  pthread_join( theThread, NULL); 
  DOUT(5, "Thread stopped!");
}

void * RobotCtr2::TmcCanCom::cyclicMessageCollectionThread(void * tmcCanCom_instance)
{
  TmcCanCom* inst = (TmcCanCom*) tmcCanCom_instance;
  
  while (inst->runThread) {
    inst->getMsgCyclic();
  } 
}

void RobotCtr2::TmcCanCom::getMsgCyclic()
{
  static CanCyclicDataFrame frame;
  static TPCANRdMsg         rcvMsgCyc;

  errno = LINUX_CAN_Read_Timeout(hcyclic, &rcvMsgCyc, 100000);
  
  if( errno ) {
    DOUT(10, "ERROR in receive thread: " << resolveCanErr(errno));
    if ( ! (errno & 0x0020 ) ) DOUT(2, "ERROR in receive thread: " << resolveCanErr(errno));
    return;
  }
  
  switch (rcvMsgCyc.Msg.ID) {
  case CAN_MSG_ID_CYCINFO:
  case CAN_MSG_ID_MOTORV:
  case CAN_MSG_ID_MOTORC:
  case CAN_MSG_ID_MOTORO:
    if (frame.add(rcvMsgCyc))
      {
	//DOUT(5, "Received a frame : " << frame.sprint());
	cyclicRingBuffer->add(frame);
	frame.init();
      }
    break;

  case CAN_MSG_ID_PING:
    pingMsgBuffer->add(PingDataMsg(rcvMsgCyc));
    break;

  case CAN_MSG_ID_CONFRET:
    confRetMsgBuffer->add(ConfRetDataMsg(rcvMsgCyc));
    break;

  default:
    DOUT(2, "Received a not known msg id.");
    break;
  }

}

bool RobotCtr2::TmcCanCom::ping(std::string * version)
{
  bool res;
  int err;
  PingDataMsg p;
  pingMsgBuffer->clear();

  reqMsgMan.MSGTYPE  = 0; 
  reqMsgMan.MSGTYPE |= MSGTYPE_RTR;
  reqMsgMan.ID       = 0x001;
  reqMsgMan.LEN      = 0;
  
  err = CAN_Write(hcyclic, &reqMsgMan);
  
  if (err) {
    DOUT(0, "ERROR in write: " << resolveCanErr(err));
    return false;
  }
  
  res = pingMsgBuffer->getNext(p, 100000);

  if (!res) return false;

  if (version != 0) *version = p.sprint();

  fw_mayer_version = p.MayerVersion;
  fw_lower_version = p.LowerVersion;

  return true;
}


void RobotCtr2::TmcCanCom::sendMotorVel(int v0, int v1, int v2, unsigned char MotorCtrMode, unsigned char kicker1, unsigned char kicker2)
{
  sndMsgMan.MSGTYPE    = 0; 
  sndMsgMan.ID         = 0x00E;
  sndMsgMan.LEN        = 8;
  for (int i=0; i<8; i++) sndMsgMan.DATA[i]=0x00;

  write10bitInBuffer( sndMsgMan.DATA, 4, 6, ABS10BIT(v0));
  write1bitInBuffer( sndMsgMan.DATA, 3, 0, BITSGN(v0));

  write10bitInBuffer( sndMsgMan.DATA, 5, 3, ABS10BIT(v1));
  write1bitInBuffer( sndMsgMan.DATA, 5, 5, BITSGN(v1));
  
  write10bitInBuffer( sndMsgMan.DATA, 6, 0, ABS10BIT(v2));
  write1bitInBuffer( sndMsgMan.DATA, 6, 2, BITSGN(v2));

  sndMsgMan.DATA[0] = MotorCtrMode;

  sndMsgMan.DATA[1] = kicker1;
  sndMsgMan.DATA[2] = kicker2;
  
  int err = CAN_Write(hcyclic, &sndMsgMan);

  if (err) {
    DOUT(0, "ERROR: " << resolveCanErr(err));
  }
  //else
  //  DOUT(5, "WROTE VEL CMD.");

}

unsigned int RobotCtr2::TmcCanCom::setCyclicSendMode(unsigned char m)
{
  bool res;
  unsigned int errcode = 0;
  ConfMsg msg( 0x01, m); 
  ConfRetDataMsg retmsg;
  confRetMsgBuffer->clear();

  int err = CAN_Write(hcyclic, &msg.Msg);

  if (err) {
    DOUT(0, "ERROR: " << resolveCanErr(err));
    errcode |= CONFEC_CANERR;
    return errcode;
  }

  // wait for the acknowledgement of the conf setting
  if (! confRetMsgBuffer->getNext(retmsg,100000)) errcode |= CONFEC_TIMEOUT;
  else errcode |= retmsg.confec;
  
  return errcode;
}


unsigned int RobotCtr2::TmcCanCom::setPIDParams(MotorID motor, unsigned int gain, unsigned int Tn, unsigned int Tv)
{
  bool res;
  unsigned int errcode = 0;
  ConfMsg msg( 0x02, (unsigned char) motor, gain, Tn, Tv); 
  ConfRetDataMsg retmsg;
  confRetMsgBuffer->clear();

  int err = CAN_Write(hcyclic, &msg.Msg);

  if (err) {
    DOUT(0, "ERROR: " << resolveCanErr(err));
    errcode |= CONFEC_CANERR;
    return errcode;
  }

  if (! confRetMsgBuffer->getNext(retmsg,100000)) errcode |= CONFEC_TIMEOUT;
  else errcode |= retmsg.confec;
  
  return errcode;
}

unsigned int RobotCtr2::TmcCanCom::setMaxMotorVel(unsigned int mmv0, unsigned int mmv1, unsigned int mmv2)
{
  bool res;
  unsigned int errcode = 0;
  ConfMsg msg( 0x03, (unsigned char) 0, mmv0, mmv1, mmv2); 
  ConfRetDataMsg retmsg;
  confRetMsgBuffer->clear();

  int err = CAN_Write(hcyclic, &msg.Msg);

  if (err) {
    DOUT(0, "ERROR: " << resolveCanErr(err));
    errcode |= CONFEC_CANERR;
    return errcode;
  }

  if (! confRetMsgBuffer->getNext(retmsg,100000)) errcode |= CONFEC_TIMEOUT;
  else errcode |= retmsg.confec;
  
  return errcode;
}

bool RobotCtr2::TmcCanCom::getLastCyclicFrame(CanCyclicDataFrame& res)
{
  return cyclicRingBuffer->getLast(res);
}

bool RobotCtr2::TmcCanCom::getNextCyclicFrame(CanCyclicDataFrame& res, unsigned long usec_patience)
{
  return cyclicRingBuffer->getNext(res, usec_patience);
}


bool RobotCtr2::TmcCanCom::getNewCyclicFrames(std::vector< CanCyclicDataFrame >& res)
{
  return  cyclicRingBuffer->getNew(res);
}



#if 0

int main() {
  unsigned int confec;

  CanCyclicDataFrame frame;
  TmcCanCom* com = 0;
  
  ofstream f("log.dat");
  
  try {
    com = new TmcCanCom();    
  }
  catch (TmcCan::TmcCanException& e) {
    std::cout << "We have an exception: " << e.what() << "\n";
    exit(0);
  }
  
  confec = com->setPIDParams(MOTOR2, 10, 80, 0);  
  DOUT(5, "CONFEC ERR: " << confec);

  com->startCyclicMessageColection();
  
  confec = com->setCyclicSendMode(CSM_ALL);
  DOUT(5, "CONFEC ERR: " << confec);

  for (int i=0; i<300; i++) 
    {
      //usleep(20000);
      //com->getLastCyclicFrame( frame );
      com->getNextCyclicFrame( frame );

      f << i << "  " 
	<< (int) frame.vel1.CycleId << "  " 
	<< frame.vel1.motorVel1 << "  " 
	<< frame.current.motorC1  << "  " 
	<< frame.vel1.vcc  << "  "
	<< frame.output.motorO1 << "  "
	<< (int) frame.cycInfo.CycleIdle_ms << "  "
	<<"\n";
      
      //DOUT(5, "Received: " << frame.sprint());

      if ( i > 10 && i < 200)
	com->sendMotorVel(0, 1023, 0 , MOTOR_CTR_MODE_PWM);
      else 
	com->sendMotorVel(0,   0, 0 , MOTOR_CTR_MODE_PWM);

      //if (i==10 ) com->sendMotorVel(0,  100, 0 , MOTOR_CTR_MODE_PID);
      //if (i==100)  com->sendMotorVel(0,  200, 0 , MOTOR_CTR_MODE_PID); 
      //if (i==200) com->sendMotorVel(0,    0, 0 , MOTOR_CTR_MODE_PID);
    }
  
  confec = com->setCyclicSendMode(CSM_NOTHING);
  DOUT(5, "CONFEC ERR: " << confec);

  com->stopCyclicMessageColection();

  com->sendMotorVel(0, 0, 0 , MOTOR_CTR_MODE_PWM);

  f.close();
  delete com;
}

#endif
