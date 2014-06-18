#include "CanMessages.h"
#include <sstream>
#include <iostream>
#include <cstring>

using namespace std;
using namespace RobotCtr2;

void write10bitInBuffer( unsigned char* array, int startbyte, int pushleft, unsigned int value)
{
  typedef union {
    unsigned char ch[2];
    unsigned short  ui;
  } trans;

  trans transformer, rmmask;

  transformer.ui = (value & 0x03FF);
  transformer.ui = (transformer.ui << pushleft);
  rmmask.ui      = ~(0x03FF << pushleft);

  for (int i=0; i<2; i++) {
    array[startbyte+i] &= rmmask.ch[1-i];
    array[startbyte+i] |= transformer.ch[1-i];
  }
 
}

unsigned short get10bitFromBuffer( const unsigned char* array, int startbyte, int pushright)
{
  unsigned int res;
  union {
    unsigned char ch[2];
    unsigned short ui;
  } transformer;

  transformer.ch[1] = array[startbyte];
  transformer.ch[0] = array[startbyte + 1];
  res = ((transformer.ui >> pushright) & 0x03FF);
  return res;
}

void write1bitInBuffer( unsigned char* array, int startbyte, int pushleft, unsigned char value)
{
  unsigned char mask = ~(0x1 << pushleft);
  unsigned char val  = value << pushleft;

  array[startbyte] &= mask;
  array[startbyte] |= val;
}

unsigned char get1bitFromBuffer( const unsigned char* array, int startbyte, int pushright)
{
  unsigned char res;
  res = ((array[startbyte] >> pushright) & 0x01);
  return res;
}


unsigned long get32bitFromBuffer( const unsigned char* array, int startbyte )
{
  union {
    unsigned char ch[4];
    unsigned long ul;
  } transformer;

  transformer.ch[3] = array[startbyte];
  transformer.ch[2] = array[startbyte+1];
  transformer.ch[1] = array[startbyte+2];
  transformer.ch[0] = array[startbyte+3];

  return transformer.ul;
}

void writeUIntInBuffer(unsigned char* array, int startbyte, unsigned int value)
{
  union {
    unsigned int i;
    unsigned char ch[2];
  } transformer;

  transformer.i = value;
  array[startbyte]   = transformer.ch[0];
  array[startbyte+1] = transformer.ch[1];
}

void writeIntInBuffer(unsigned char* array, int startbyte, unsigned int value)
{
  union {
    int i;
    unsigned char ch[2];
  } transformer;
  
  transformer.i = value;
  array[startbyte]   = transformer.ch[0];
  array[startbyte+1] = transformer.ch[1];
}

int getIntFromBuffer(const unsigned char* array, int startbyte)
{
  union {
    int i;
    unsigned char ch[2];
  } transformer;
  transformer.ch[0] = array[startbyte];
  transformer.ch[1] = array[startbyte+1];

  return transformer.i;  
}

CycleInfoDataMsg::CycleInfoDataMsg(const TPCANRdMsg& _rcvMsg) 
{
  setData(_rcvMsg);
}

CycleInfoDataMsg::CycleInfoDataMsg()
{
  ;
}

void CycleInfoDataMsg::setData(const TPCANRdMsg& _rcvMsg) 
{ 
  CanTimeStamp     = _rcvMsg.dwTime;
  CycleId          = _rcvMsg.Msg.DATA[7];
  CycleCounter     = get32bitFromBuffer( _rcvMsg.Msg.DATA, 4);
  CycleDuration_ms = get10bitFromBuffer( _rcvMsg.Msg.DATA, 2, 0);
  CycleIdle_ms     = get10bitFromBuffer( _rcvMsg.Msg.DATA, 1, 2) *(get1bitFromBuffer( _rcvMsg.Msg.DATA, 1, 4) ? -1 : 1 );
  
}

std::string CycleInfoDataMsg::sprint()
{
  std::stringstream res;
  res << "Cycle Info: " 
      << " CanTime: " << CanTimeStamp 
      << " ID: "      << (unsigned int) CycleId 
      << " CycCounter: " << CycleCounter 
      << " CycleDuration_ms: " << CycleDuration_ms
      << " CycleIdle_ms: " << CycleIdle_ms 
      << "  ";
  return res.str();
}

MotorVDataMsg::MotorVDataMsg(const TPCANRdMsg& _rcvMsg)
{
  setData(_rcvMsg);
}

MotorVDataMsg::MotorVDataMsg()
{
  ;
}

void MotorVDataMsg::setData(const TPCANRdMsg& _rcvMsg)
{
  CanTimeStamp     = _rcvMsg.dwTime;
  CycleId          = _rcvMsg.Msg.DATA[7];
  motorVel0        = get10bitFromBuffer( _rcvMsg.Msg.DATA, 3, 6) * (get1bitFromBuffer( _rcvMsg.Msg.DATA, 2, 0) ? -1 : 1 );
  motorVel1        = get10bitFromBuffer( _rcvMsg.Msg.DATA, 4, 3) * (get1bitFromBuffer( _rcvMsg.Msg.DATA, 4, 5) ? -1 : 1 );
  motorVel2        = get10bitFromBuffer( _rcvMsg.Msg.DATA, 5, 0) * (get1bitFromBuffer( _rcvMsg.Msg.DATA, 5, 2) ? -1 : 1 );
  vcc              = get10bitFromBuffer( _rcvMsg.Msg.DATA, 1, 1);
  herr0            = get1bitFromBuffer ( _rcvMsg.Msg.DATA, 1, 3);
  herr1            = get1bitFromBuffer ( _rcvMsg.Msg.DATA, 1, 4);
  herr2            = get1bitFromBuffer ( _rcvMsg.Msg.DATA, 1, 5);
}

std::string MotorVDataMsg::sprint()
{
  std::stringstream res;
  
  res << "VEL1 " 
      << " CanTime: " << CanTimeStamp  
      << " ID: "      << (unsigned int) CycleId 
      << " motorVel0: " << motorVel0
      << " motorVel1: " << motorVel1
      << " motorVel2: " << motorVel2
      << " vcc: "       << vcc
      << " herr: " << herr0 << " " << herr1 << "  " << herr2 << "  ";

  return res.str();
}



//-- MotorCDataMsg ----------------------------------------------------------------------------

MotorCDataMsg::MotorCDataMsg(const TPCANRdMsg& _rcvMsg)
{
  setData(_rcvMsg);
}

MotorCDataMsg::MotorCDataMsg()
{
  ;
}

void MotorCDataMsg::setData(const TPCANRdMsg& _rcvMsg)
{
  CanTimeStamp   = _rcvMsg.dwTime;
  CycleId        = _rcvMsg.Msg.DATA[7];
  motorC0        = get10bitFromBuffer( _rcvMsg.Msg.DATA, 3, 4);
  motorC1        = get10bitFromBuffer( _rcvMsg.Msg.DATA, 4, 2);
  motorC2        = get10bitFromBuffer( _rcvMsg.Msg.DATA, 5, 0);
}

std::string MotorCDataMsg::sprint()
{
  std::stringstream res;
  
  res << "CURRENT " 
      << " CanTime: " << CanTimeStamp  
      << " ID: "      << (unsigned int) CycleId 
      << " motorC0: " << motorC0
      << " motorC1: " << motorC1
      << " motorC2: " << motorC2;
  return res.str();
}
//------------------------------------------------------------------------------



//-- MotorODataMsg ----------------------------------------------------------------------------

MotorODataMsg::MotorODataMsg(const TPCANRdMsg& _rcvMsg)
{
  setData(_rcvMsg);
}

MotorODataMsg::MotorODataMsg()
{
  ;
}

void MotorODataMsg::setData(const TPCANRdMsg& _rcvMsg)
{
  CanTimeStamp   = _rcvMsg.dwTime;
  CycleId        = _rcvMsg.Msg.DATA[7];
  motorO0        = (int) get10bitFromBuffer( _rcvMsg.Msg.DATA, 3, 6) *  (get1bitFromBuffer(_rcvMsg.Msg.DATA, 2, 0 ) ? -1 : 1);
  motorO1        = (int) get10bitFromBuffer( _rcvMsg.Msg.DATA, 4, 3) *  (get1bitFromBuffer(_rcvMsg.Msg.DATA, 4, 5 ) ? -1 : 1);
  motorO2        = (int) get10bitFromBuffer( _rcvMsg.Msg.DATA, 5, 0) *  (get1bitFromBuffer(_rcvMsg.Msg.DATA, 5, 2 ) ? -1 : 1);
}

std::string MotorODataMsg::sprint()
{
  std::stringstream res;
  
  res << "CURRENT " 
      << " CanTime: " << CanTimeStamp  
      << " ID: "      << (unsigned int) CycleId 
      << " motorO0: " << motorO0
      << " motorO1: " << motorO1
      << " motorO2: " << motorO2;
  return res.str();
}
//------------------------------------------------------------------------------

//-- PingDataMsg ----------------------------------------------------------------------------

PingDataMsg::PingDataMsg(const TPCANRdMsg& _rcvMsg)
{
  setData(_rcvMsg);
}

PingDataMsg::PingDataMsg()
{
  ;
}

void PingDataMsg::setData(const TPCANRdMsg& _rcvMsg)
{
  MayerVersion   = _rcvMsg.Msg.DATA[6];
  LowerVersion   = _rcvMsg.Msg.DATA[7];
  memcpy(VersionString, _rcvMsg.Msg.DATA, 6);
  VersionString[6] = 0;
}

std::string PingDataMsg::sprint()
{
  std::stringstream res;
  
  res << VersionString 
      << "  Verion: " << (int) MayerVersion  
      << "."          << (int) LowerVersion;
  return res.str();
}
//------------------------------------------------------------------------------

//-- ConfRetDataMsg ----------------------------------------------------------------------------

ConfRetDataMsg::ConfRetDataMsg(const TPCANRdMsg& _rcvMsg)
{
  setData(_rcvMsg);
}

ConfRetDataMsg::ConfRetDataMsg()
{
  ;
}

void ConfRetDataMsg::setData(const TPCANRdMsg& _rcvMsg)
{
  ConfId = _rcvMsg.Msg.DATA[7];
  confec = _rcvMsg.Msg.DATA[6];
  
  Param1 = getIntFromBuffer(_rcvMsg.Msg.DATA, 4);
  Param2 = getIntFromBuffer(_rcvMsg.Msg.DATA, 2);
  Param3 = getIntFromBuffer(_rcvMsg.Msg.DATA, 0);
}

std::string ConfRetDataMsg::sprint()
{
  std::stringstream res;
  
  res << "ConfId: " << (int) ConfId
      << " confec: " << (int) confec  
      << " P1: "     << Param1
      << " P2: "     << Param2
      << " P3: "     << Param3;
  return res.str();
}
//------------------------------------------------------------------------------

//---CyclicDataFrame---------------------------------------------------------------------------

CanCyclicDataFrame::CanCyclicDataFrame()
{
  init();
}

void CanCyclicDataFrame::init()
{
  cycInfo_set = vel_set = current_set = output_set = false;
}

bool CanCyclicDataFrame::ready()
{
  return (cycInfo_set && vel_set && current_set && output_set);
}

int  CanCyclicDataFrame::msgCycId()
{
  if (!cycInfo_set) {
    return -1;
  }
  return cycInfo.CycleId;
}

bool CanCyclicDataFrame::add(const TPCANRdMsg& _rcvMsg)
{
  // TODO was passiert wenn Nachrichten ausgelassen werden -> ID prüfen
  switch (_rcvMsg.Msg.ID) {
  case 0x002:
    cycInfo.setData(_rcvMsg);    
    cycInfo_set = true;
    break;
  case 0x003:
    if (!cycInfo_set) return true;
    vel.setData(_rcvMsg);
    vel_set = true;
    break;
  case 0x004:
    if (!cycInfo_set) return true;
    current.setData(_rcvMsg);
    current_set = true;
    break;
  case 0x005:
    if (!cycInfo_set) return true;
    output.setData(_rcvMsg);
    output_set = true;
    break;
  default:

    std::cerr << "RECEIVED an not known FRAME\n\n";

    break;
  }

  return ready();
}

std::string CanCyclicDataFrame::sprint()
{
  std::string res;
  
  res = "CyclicDataContent: ";
  if (cycInfo_set) res += cycInfo.sprint();
  if (vel_set)     res += vel.sprint();
  if (current_set) res += current.sprint();

  return res;
}


ConfMsg::ConfMsg()
{
  Msg.MSGTYPE    = 0; 
  Msg.ID         = CAN_MSG_ID_S_CONF;
  Msg.LEN        = 8;
  for (int i=0; i<8; i++) Msg.DATA[i]=0x00;
}

ConfMsg::ConfMsg(unsigned char ConfId, unsigned char Param1, unsigned int Param2, unsigned int Param3, unsigned int Param4)
{
  Msg.MSGTYPE    = 0; 
  Msg.ID         = CAN_MSG_ID_S_CONF;
  Msg.LEN        = 8;
  for (int i=0; i<8; i++) Msg.DATA[i]=0x00;
  setData(ConfId, Param1, Param2, Param3, Param4);
}


void ConfMsg::setData(unsigned char ConfId, unsigned char Param1, unsigned int Param2,  unsigned int Param3,  unsigned int Param4)
{
  Msg.DATA[7] = ConfId;
  Msg.DATA[6] = Param1;
  writeUIntInBuffer(Msg.DATA, 4, Param2);
  writeUIntInBuffer(Msg.DATA, 2, Param3);
  writeUIntInBuffer(Msg.DATA, 0, Param4);
}

// // -------------- Cyclic Data Ring Buffer ----------------------------------------------
// CanCyclicDataRingBuffer::CanCyclicDataRingBuffer(unsigned int _buffer_size ) throw (std::bad_alloc)
//   : buffer_size( _buffer_size ), windex(0), rindex(0)
// {
//   buffer = 0;
//   buffer = new CanCyclicDataFrame[buffer_size];

//   buffermutex = (pthread_mutex_t) PTHREAD_MUTEX_INITIALIZER;
// }


// CanCyclicDataRingBuffer::~CanCyclicDataRingBuffer()
// {
//   if (buffer!=0) delete[] buffer;
// }

// void CanCyclicDataRingBuffer::add(const CanCyclicDataFrame& newFrame)
// {
//   pthread_mutex_lock( &buffermutex );
//   windex = (windex + 1) % buffer_size;
//   buffer[windex] = newFrame;
//   pthread_mutex_unlock( &buffermutex );
// }

// bool CanCyclicDataRingBuffer::getLast(CanCyclicDataFrame& res)
// {
//   bool isnewdata = true;

//   pthread_mutex_lock( &buffermutex );
//   if (windex == rindex) isnewdata = false;
//   res = buffer[windex];
//   rindex = windex;
//   pthread_mutex_unlock( &buffermutex );

//   return isnewdata;
// }

// void CanCyclicDataRingBuffer::getAll(std::vector< CanCyclicDataFrame >& res)
// {
//   pthread_mutex_lock( &buffermutex );
//   res.clear();
//   for (unsigned int i=0; i<buffer_size; i++)
//     {
//       res.push_back( buffer[(windex + i) % buffer_size]);
//     }
//   rindex = windex;
//   pthread_mutex_unlock( &buffermutex );
// }


// bool CanCyclicDataRingBuffer::getNew(std::vector< CanCyclicDataFrame >& res)
// {
//   bool isnewdata = true;  
//   pthread_mutex_lock( &buffermutex );
//   res.clear();  
//   if (rindex == windex) {
//     isnewdata = false;
//   }
//   else {      
//     do {
//       rindex = (rindex +1) % buffer_size;
//       res.push_back( buffer[rindex]);
//     } while (rindex != windex);
//   }
//   pthread_mutex_unlock( &buffermutex );
  
//   return isnewdata;
// }




