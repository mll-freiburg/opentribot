#include "bvcanopenobject.h"
// ---------------------------------------------------
// ---------------------------------------------------
// -- ctor -------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
BVCanOpenObject::BVCanOpenObject(){
}
// ---------------------------------------------------
// ---------------------------------------------------
// -- ctor -------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
BVCanOpenObject::BVCanOpenObject(string aName,
                                 int anIndex,
                                 int aSubIndex,
                                 enum typeEnum aType,
                                 enum accessEnum anAccess)
           :name(aName), 
            index(anIndex),
            subIndex(aSubIndex),
	    type(aType),
            access(anAccess){
  init();
}

// ---------------------------------------------------
// ---------------------------------------------------
// -- init -------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
void BVCanOpenObject::init(){
  // ........................................................
  // Messages from PC to EPOS
  // ........................................................
  // pc send a read SDO to epos
  pc2EposRead            = 0x40;
  // pc send a write SDO to epos for writing 1 byte to epos
  pc2EposWrite1Byte      = 0x2F;
  // pc send a write SDO to epos for writing 2 byte to epos
  pc2EposWrite2Byte      = 0x2B;
  // pc send a write SDO to epos for writing 4 byte to epos
  pc2EposWrite4Byte      = 0x23;
  // NOT DEFINED
  pc2EposWriteNotDefined = 0x22;
  // ........................................................
  // ........................................................
  // ........................................................
  // Messages from EPOS to PC
  // ........................................................
  // epos send a SDO with 1 byte data to PC
  epos2PC1Byte  = 0x4F;
  // epos send a SDO with 2 byte data to PC
  epos2PC2Byte  = 0x4B;
  // epos send a SDO with 4 byte data to PC
  epos2PC4Byte  = 0x43;
  // epos send a SDO in response to a write SDO from pc
  epos2PCAck    = 0x60;
  // ........................................................
  // ........................................................
 
}
// ---------------------------------------------------
// ---------------------------------------------------
// -- value ------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
int BVCanOpenObject::getCommandSpecifier( enum directionEnum aDiresction, enum readWriteEnum aReadWrite ){

  int ret;
  switch(aDiresction){
  case pcToEpos:
    switch(aReadWrite){
    case read:
        ret=pc2EposRead;
        break;
    case write:
        switch(type){
        case UNSIGNED8:
          ret=pc2EposWrite1Byte;
          break;
        case UNSIGNED16:
          ret=pc2EposWrite2Byte;
          break;
        case UNSIGNED32:
          ret=pc2EposWrite4Byte;
          break;
        case INTEGER8:
          ret=pc2EposWrite1Byte;
          break;
        case INTEGER16:
          ret=pc2EposWrite2Byte;
          break;
        case INTEGER32:
          ret=pc2EposWrite4Byte;
          break;
        default:
          BV_WARNING("BVCanOpenDialog::toggleAction ERROR ");
          break;
        }
        break;
      default:
        BV_WARNING("BVCanOpenObject::getCommandSpecifier unknown operation");
        break;
    }
  case eposToPc:
    break;
  default:
    BV_WARNING("BVCanOpenObject::getCommandSpecifier unknown diresction");
    break;
  }

  return ret;
}
// ---------------------------------------------------
// ---------------------------------------------------
// -- prepareToSend ----------------------------------
// @param1, aNodeID is the canNodeID, 0x02,0x04,0x08
// ---------------------------------------------------
// ---------------------------------------------------
void BVCanOpenObject::prepareToSend(int aNodeID,enum readWriteEnum aReadWrite , int* aval){
  int canMessagetype = 0x00; // standard message
  int length = 0x08;
  int canID=0x600 + aNodeID ;
   
  msg.setCanID(canID );
  msg.setMSGTYPE(canMessagetype);
  msg.setLEN(length);

  int cs = getCommandSpecifier( pcToEpos , aReadWrite );

  int byte1 = index    & 0xFF;
  int byte2 = index    & 0xFF00;
  int byte3 = subIndex & 0xFF;
  int byte4 = (*aval)  & 0xFF;
  int byte5 = (*aval)  & 0xFF00;
  int byte6 = (*aval)  & 0xFF0000;
  int byte7 = (*aval)  & 0xFF000000;
 
  msg.setDATA( 0 , cs & 0xFF );
  msg.setDATA( 1 , byte1 );
  msg.setDATA( 2 , byte2 >> 8 );
  msg.setDATA( 3 , byte3 );
  msg.setDATA( 4 , byte4 );
  msg.setDATA( 5 , byte5 >> 8);
  msg.setDATA( 6 , byte6 >> 16);
  msg.setDATA( 7 , byte7 >> 24 );

  value1(&msg);
  
}

// ---------------------------------------------------
// ---------------------------------------------------
// -- prepareToSend ----------------------------------
// @param1, aNodeID is the canNodeID, 0x02,0x04,0x08
// ---------------------------------------------------
// ---------------------------------------------------
void BVCanOpenObject::prepareToAnswerFromSimAgent(int aNodeID,enum readWriteEnum aReadWrite , int* aval){
  int canMessagetype = 0x00; // standard message
  int length = 0x08;
  int canID=0x580 + aNodeID ;
  msg.setCanID(canID );
  msg.setMSGTYPE(canMessagetype);
  msg.setLEN(length);

  int cs = getCommandSpecifier( pcToEpos , aReadWrite );

  int byte1 = index    & 0xFF;
  int byte2 = index    & 0xFF00;
  int byte3 = subIndex & 0xFF;
  int byte4 = (*aval)  & 0xFF;
  int byte5 = (*aval)  & 0xFF00;
  int byte6 = (*aval)  & 0xFF0000;
  int byte7 = (*aval)  & 0xFF000000;
 
  msg.setDATA( 0 , cs & 0xFF );
  msg.setDATA( 1 , byte1 );
  msg.setDATA( 2 , byte2 >> 8 );
  msg.setDATA( 3 , byte3 );
  msg.setDATA( 4 , byte4 );
  msg.setDATA( 5 , byte5 >> 8);
  msg.setDATA( 6 , byte6 >> 16);
  msg.setDATA( 7 , byte7 >> 24 );

  value1(&msg);
  
}
// ---------------------------------------------------
// ---------------------------------------------------
// -- setMsg -----------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
void BVCanOpenObject::setMsg( BVMessageTPCANMsg* amsg){
  msg=*amsg;
  value1(&msg);
}
// ---------------------------------------------------
// ---------------------------------------------------
// -- getMsg -----------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
BVMessageTPCANMsg BVCanOpenObject::getMsg(){
  return msg;
}
// ---------------------------------------------------
// ---------------------------------------------------
// -- getStrValue ------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
string BVCanOpenObject::getStrValue(){
  return strValue;
}
// ---------------------------------------------------
// ---------------------------------------------------
// -- value ------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
void BVCanOpenObject::value1(BVMessageTPCANMsg *amsg ){

    int canID        = (int)( amsg->getCanID()   );
    int messasgeType = (int)( amsg->getMSGTYPE() );
    int length       = (int)( amsg->getLEN()     );
    int data0        = (int)amsg->getDATA(0);
    int data1        = (int)amsg->getDATA(1);
    int data2        = (int)amsg->getDATA(2);
    int data3        = (int)amsg->getDATA(3);
    int data4        = (int)amsg->getDATA(4);
    int data5        = (int)amsg->getDATA(5);
    int data6        = (int)amsg->getDATA(6);
    int data7        = (int)amsg->getDATA(7);

    stringstream sstr;
    string ret;
    switch(type){
    case VISIBLE_STRING:
      BV_WARNING("BVCanOpenObject::value DO NOTHING");
      break;
    case UNSIGNED8:
      val.valU32  = data4;
      strValue = sstr.str(); 
      sstr << val.valU32;
      break;
    case UNSIGNED16:
      val.valU32 = (data5 << 8) + data4;
      sstr << val.valU32;
      strValue = sstr.str(); 
      break;
    case UNSIGNED32:
      val.valU32 = (data7 << 24) + (data6 << 16) +( data5 << 8) + (data4);
      sstr << val.valU32;
      strValue = sstr.str(); 
      break;
    case INTEGER8:
      {
      //             unsigned  signed
      // 0111 1111  =127       127
      // 0000 0010  =2         2 
      // 0000 0001  =1         1
      // 0000 0000  =0         0
      // 1111 1111  =−1        255
      // 1111 1110  =−2        254
      // 1000 0001  =−127      129
      // 1000 0000  =−128      128

      // byte 
      // Signed: −128 to +127
      // Unsigned: 0 to +255
      int v = (data4);
      v= v<=0x7F ? v : -(0xFF-v) ;
      val.val32 = v;
      sstr << val.val32;
      strValue = sstr.str(); 
      }
      break;
    case INTEGER16: 
      {
      // 2 bytes
      //Signed: −32,768 to +32,767
      //Unsigned: 0 to +65,535
      int v = (data5 << 8) + (data4) ;
      v= v<=0x7FFF ? v : -(0xFFFF-v) ;
      val.val32 = v;
      sstr << val.val32;
      strValue = sstr.str(); 
      }
      break;
    case INTEGER32:
      {
      // 4 bytes
      //Signed: −2,147,483,648 to +2,147,483,647
      //Unsigned: 0 to +4,294,967,295
      int v =  (data7 << 24) + (data6 << 16) +( data5 << 8) + (data4);
      v= v<=0x7FFFFFFF ? v : -(0xFFFFFFFF-v) ;
      val.val32 = (data7 << 24) + (data6 << 16) +( data5 << 8) + (data4);
      sstr << val.val32;
      strValue = sstr.str(); 
      }
      break;
    default:
      BV_WARNING("BVCanOpenObject::value UNKNOWN type");
      break;
    }

}
// ---------------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------
// ---------------------------------------------------


