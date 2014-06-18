#ifndef BV_MESSAGE
#define BV_MESSAGE

#include "bvdebug.h"
#include "bvbinaryio.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
typedef unsigned long DWORD;
typedef int WORD;
typedef unsigned char BYTE;

#include "math.h"
#include "bvmessagebase.h"

#ifdef USE_CAN
#include "pcan.h"
#endif

#define ID_BVMessageTPCANMsg                    49

using namespace std;





class BVMessageTPCANMsg : public BVMessageBase{
public:
  BVMessageTPCANMsg( int aSenderID , int aReceiverID,
                          DWORD acanID, BYTE aMSGTYPE , BYTE aLEN ,
                          BYTE aDATA0,BYTE aDATA1,BYTE aDATA2,BYTE aDATA3,
                          BYTE aDATA4,BYTE aDATA5,BYTE aDATA6,BYTE aDATA7 )
                          :  canID(acanID),
                             MSGTYPE(aMSGTYPE),
                             LEN(aLEN)
  {
    id = ID_BVMessageTPCANMsg;
    name = "BVMessageTPCANMsg";
    senderID = aSenderID;
    receiverID = aReceiverID;
    DATA[0]=( aDATA0);
    DATA[1]=( aDATA1);
    DATA[2]=( aDATA2);
    DATA[3]=( aDATA3);
    DATA[4]=( aDATA4);
    DATA[5]=( aDATA5);
    DATA[6]=( aDATA6);
    DATA[7]=( aDATA7);
  }

  BVMessageTPCANMsg(){
    id = ID_BVMessageTPCANMsg;
    name = "BVMessageTPCANMsg" ;
  }
  /** writeCostum the object to stream */
  void writeCostum( ostream* astream ){
    *astream < canID < MSGTYPE < LEN < DATA[0] < DATA[1] <DATA[2] <DATA[3] <DATA[4] <DATA[5] <DATA[6] < DATA[7] ;
  }

  /** readCostum object from stream  */
  void readCostum(istream* astream){
    *astream > canID > MSGTYPE > LEN > DATA[0] > DATA[1] >DATA[2] >DATA[3] >DATA[4] >DATA[5] >DATA[6] > DATA[7] ;
  }

  /** access to private data members */
  string getName()   { return name; }
  DWORD  getCanID()  { return canID; }
  BYTE getMSGTYPE()  { return MSGTYPE; }
  BYTE getLEN()  { return LEN; }
  BYTE getDATA(int index)  { return DATA[index] ; }

  void setCanID( DWORD acanID){ canID=acanID;}
  void setMSGTYPE(BYTE  aMSGTYPE){ MSGTYPE=aMSGTYPE;}
  void setLEN(BYTE  aLEN){ LEN=aLEN;}
  void setDATA(int index , BYTE aval)  { DATA[index]=aval ; }

private:
  /** a name */
  string name;
  DWORD canID;              // 11/29 bit code
  BYTE  MSGTYPE;         // bits of MSGTYPE_*
  BYTE  LEN;             // count of data bytes (0..8)
  BYTE  DATA[8]; 
};



#endif

