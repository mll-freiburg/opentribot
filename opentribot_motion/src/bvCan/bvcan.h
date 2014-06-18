#ifndef BV_CAN
#define BV_CAN





#include "bvdebug.h"
#include <string.h>
#include "bvfd.h"
#include "bvmessage.h"

#include <fcntl.h>
#include <cerrno>
#ifdef USE_CAN
#include <libpcan.h>
#endif

//#define DEFAULT_NODE "/dev/pcan0"

#define PROCFILE "/proc/pcan"     // where to get information
#define MAX_LINE_LEN 255          // to store a line of text
#define DEVICE_PATH "/dev/pcan"   // + Minor = real device path
#define LOCAL_STRING_LEN 64       // length of internal used strings





//--------------defined in libpcan.c-----------------------
typedef struct
{
  char szVersionString[LOCAL_STRING_LEN];
  char szDevicePath[LOCAL_STRING_LEN];
  int  nFileNo;
} PCAN_DESCRIPTOR;

/**
--------------defined in pcan.h---------------------------
typedef struct
{
  DWORD ID;              // 11/29 bit code
  BYTE  MSGTYPE;         // bits of MSGTYPE_*
  BYTE  LEN;             // count of data bytes (0..8)
  BYTE  DATA[8];         // data bytes, up to 8
} TPCANMsg;              // for PCAN_WRITE_MSG
----------------------------------------------------------
*/

// -----------------------------------------------------------
// -----------------------------------------------------------
// -----------------------------------------------------------
// DWORD CAN_Init(HANDLE hHandle, WORD wBTR0BTR1, int nCANMsgType)
// -----------------------------------------------------------
// -----------------------------------------------------------
// -----------------------------------------------------------
// init to a user defined bit rate
// 2nd param : initializes the CAN hardware with the BTR0 + BTR1 constant "CAN_BAUD_."
// parameter wBTR0BTR1
// bitrate codes of BTR0/BTR1 registers
// #define CAN_BAUD_1M     0x0014  //   1 MBit/s
// #define CAN_BAUD_500K   0x001C  // 500 kBit/s
// #define CAN_BAUD_250K   0x011C  // 250 kBit/s
// #define CAN_BAUD_125K   0x031C  // 125 kBit/s
// #define CAN_BAUD_100K   0x432F  // 100 kBit/s
// #define CAN_BAUD_50K    0x472F  //  50 kBit/s
// #define CAN_BAUD_20K    0x532F  //  20 kBit/s
// #define CAN_BAUD_10K    0x672F  //  10 kBit/s
// #define CAN_BAUD_5K     0x7F7F  //   5 kBit/s
// 3rd param: must be filled with "CAN_INIT_TYPE_.."
// #define CAN_INIT_TYPE_EX    0x01    //Extended Frame
// #define CAN_INIT_TYPE_ST    0x00    //Standart Frame

// The default initialisation, e.g. CAN_Init is not called,
//  is 500 kbit/sec and extended frames.
// -----------------------------------------------------------
// -----------------------------------------------------------
// -----------------------------------------------------------
// -----------------------------------------------------------



using namespace std;

class BVCan : public BVfd{
public:
  /** ctor */
  BVCan();
  /** dtor */
  ~BVCan();
  void initializeCan( );
  void resetFilter( );
  int write( void *param);
  int read( void* param );
private:
  // HANDLE is defined in:
  // libpcan.h as: #define HANDLE void *
#ifdef USE_CAN
HANDLE h;
  string device;
  char info[VERSIONSTRING_LEN];
  __u16 wBTR0BTR1 ;
  int   canMsgType ;
#endif

  
};

#endif
