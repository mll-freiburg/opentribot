#include "bvcan.h"

/** ctor */
BVCan::BVCan(){

}

BVCan::~BVCan(){

#ifdef USE_CAN
  if (!h){
    CAN_Close(h);
    BV_DEBUGINFO4("BVCan::dtor close can");
  }
#endif
  
  
  
}
void BVCan::resetFilter(    ){
#ifdef USE_CAN
  
   CAN_ResetFilter(h);
#endif   
   
 }
void BVCan::initializeCan(    ){

#ifdef USE_CAN
  
  WORD awBTR0BTR1=0x0014;
 int aCANMsgType=0;


   h = NULL;

  errno = 0;


  device = string("/dev/pcan32");
  //  open the device
  //    //  O_RDWR which request opening the file read/write
  h = LINUX_CAN_Open((char*)(device.c_str()), O_RDWR);
  if (h==0)
  {
      device = string("/dev/pcan0");
      h = LINUX_CAN_Open((char*)(device.c_str()), O_RDWR);
  }
  if(h==0){
      cerr<< "PCan_Device nicht gefunden !!!"<<endl;
      exit(37);
  }
  
  
  
  if(h){
    PCAN_DESCRIPTOR *desc= (PCAN_DESCRIPTOR *)h;
    BV_ASSERT( desc->nFileNo>0 );
    BV_DEBUGINFO4NNL("BVCan::ctor device=[" << desc->szDevicePath << "]");
    BV_DEBUGINFO4C(" fd=[" << desc->nFileNo << "]" << endl);
    setfd(desc->nFileNo);
    CAN_Status(h);
    // -----------------------------------------------------------
    // returns a text string with driver version info
    errno = CAN_VersionInfo(h, info);
    if (!errno){
      string infostr( string(info) );
      BV_DEBUGINFO4("BVCan::ctor driver version=[" << infostr << "]");
    }
    else
      perror("BVCan::ctor CAN_VersionInfo()");

    wBTR0BTR1 = 0x401c; 
    if(LINUX_CAN_BTR0BTR1(h,wBTR0BTR1)==0)
      BV_WARNING("BVCan::ctor 0x" << hex << wBTR0BTR1 << dec << " is not possible" );	 
    else
      BV_DEBUGINFO4("BVCan::ctor baudrate changes to 0x" << hex << wBTR0BTR1 << dec );	      
  } 
  else
    BV_DEBUGINFO4("BVCan::ctor cannot open the can device");	  

  BV_WARNING("BVCan::initializeCan .....................");
  if(h){
    // init to a user defined bit rate
    //nExtended = CAN_INIT_TYPE_ST;
    //wBTR0BTR1 = CAN_BAUD_500K;
    canMsgType = aCANMsgType;
    wBTR0BTR1 = awBTR0BTR1;


    errno = CAN_Init(h, wBTR0BTR1, canMsgType);
    if (errno)
      perror("BVCan::initializeCan CAN_Init failed");
    else
      BV_WARNING("BVCan::initializeCan baudrate=0x" << hex << wBTR0BTR1 << dec << " msgType=" << canMsgType );
  }
  else
    BV_WARNING("BVCan::initializeCan can HANDLE is NULL");

  
  #endif

  
  
}

// writes a message to the CAN bus. If the write queue is full the current
// write blocks until either a message is sent or a error occured.
// 
// CAN_Write()
//  writes a message to the CAN bus. If the write queue is full the current
//  write blocks until either a message is sent or a error occured.

int BVCan::write( void *param){
  // IMPORTANT : msg should be a pointer to a string
  BVMessageTPCANMsg message;
  message.stringToObject(*(string*)param);
  int count;     

  #ifdef USE_CAN
  TPCANMsg canmsg;
  canmsg.ID=message.getCanID();
  canmsg.MSGTYPE=message.getMSGTYPE();
  canmsg.LEN=message.getLEN();
  canmsg.DATA[0]=message.getDATA(0);
  canmsg.DATA[1]=message.getDATA(1);
  canmsg.DATA[2]=message.getDATA(2);
  canmsg.DATA[3]=message.getDATA(3);
  canmsg.DATA[4]=message.getDATA(4);
  canmsg.DATA[5]=message.getDATA(5);
  canmsg.DATA[6]=message.getDATA(6);
  canmsg.DATA[7]=message.getDATA(7);
  
  
  errno = CAN_Write(h, &canmsg);
  if (errno){
    perror("BVCan::write CAN_Write()");
    BV_WARNING("BVCan::write .....E R R O R ");
  }
  else{
    count = (sizeof( canmsg.ID )) +
             sizeof( canmsg.MSGTYPE )+
             sizeof( canmsg.LEN )+
             canmsg.LEN * sizeof( canmsg.DATA[0] );
    //BV_DEBUGINFO4NNL("BVCan::write ID=" << hex << (int)(canmsg.ID) );
    //BV_DEBUGINFO4C(" MSGTYPE=[" << (int)(canmsg.MSGTYPE) << "] " );
    //BV_DEBUGINFO4C("LEN=[" << (int)(canmsg.LEN) << "] " );
    //BV_DEBUGINFO4C("DATA=[" << (int)(canmsg.DATA[0]) << "]" );
    //BV_DEBUGINFO4C("[" << (int)(canmsg.DATA[1]) << "]" );
    //BV_DEBUGINFO4C("[" << (int)(canmsg.DATA[2]) << "]" );
    //BV_DEBUGINFO4C("[" << (int)(canmsg.DATA[3]) << "]" );
    //BV_DEBUGINFO4C("[" << (int)(canmsg.DATA[4]) << "]" );
    //BV_DEBUGINFO4C("[" << (int)(canmsg.DATA[5]) << "]" );
    //BV_DEBUGINFO4C("[" << (int)(canmsg.DATA[6]) << "]" );
    //BV_DEBUGINFO4C("[" << (int)(canmsg.DATA[7])<< "]" << dec << endl );
  }
  
#endif
  return count;
  
  
  
}


int BVCan::read( void *param ){
  // IMPORTANT : msg should be a pointer to a string	


  int count=0;
  
#ifdef USE_CAN
  
  TPCANMsg canmsg;	
  errno = CAN_Read( h, &canmsg);
 
  if (errno){
    BV_WARNING("BVCan::read XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    perror("BVCan::read CAN_Read() return ERROR, means that I did not read anything at all");
    BV_WARNING("BVCan::read XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxXXXXXXXXX");
    BV_ASSERT(0);
  }
  else{
    if ( (int)(canmsg.MSGTYPE) == MSGTYPE_STATUS){
      __u32 status = CAN_Status(h);
      BV_WARNING("BVCan::read E R R O R : peak insert a ERROR message in Q with messageType=0x80 CAN_Status(h)=[" << status <<"]");
    }

    count = (sizeof( canmsg.ID )) +
	        sizeof( canmsg.MSGTYPE )+
	        sizeof( canmsg.LEN )+
	        canmsg.LEN * sizeof( canmsg.DATA[0] );
    BVMessageTPCANMsg mes(0,0,canmsg.ID ,canmsg.MSGTYPE,canmsg.LEN,
		              canmsg.DATA[0],canmsg.DATA[1],canmsg.DATA[2],
			      canmsg.DATA[3],canmsg.DATA[4],canmsg.DATA[5],
			      canmsg.DATA[6],canmsg.DATA[7]);
    *((string*)param) =  mes.objectToString();
  }
#endif
  return count;

}
