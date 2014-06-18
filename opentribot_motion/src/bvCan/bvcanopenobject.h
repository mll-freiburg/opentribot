#ifndef BV_CAN_OPEN_OBJECT
#define BV_CAN_OPEN_OBJECT

#include "bvdebug.h"
#include "bvmessage.h"
using namespace std;

class BVCanOpenObject{
public:
  enum readWriteEnum{ read=0,write};
  enum directionEnum{pcToEpos=0,eposToPc};
  enum typeEnum { VISIBLE_STRING=1 , UNSIGNED8 , UNSIGNED16 , UNSIGNED32 , INTEGER8 , INTEGER16  , INTEGER32 };
  enum accessEnum { RW=1 , RO , CONST};
public:
  BVCanOpenObject(string aName, int anIndex , int aSubIndex , enum typeEnum aType , enum accessEnum anAccess);
  BVCanOpenObject();

  int getCommandSpecifier( enum directionEnum aDiresction, enum readWriteEnum aReadWrite );
  void prepareToSend(int aNodeID,enum readWriteEnum aReadWrite , int* aval    );
  void prepareToAnswerFromSimAgent(int aNodeID,enum readWriteEnum aReadWrite , int* aval);
  
public:
  int getCanOpenObjectID(){ return canOpenObjectID;}

  string getName(){ return name;}
  enum typeEnum getType(){ return type;}

  void setRowInTable(int arow){rowInTable=arow;}
  int getRowInTable(){return rowInTable;}


  int getVal32(){ return val.val32;}
  unsigned int getValU32(){return val.valU32;}

  string getStrValue();

  BVMessageTPCANMsg getMsg();
  void setMsg( BVMessageTPCANMsg* amsg);
private:
  void init();
 public:
  void value1(BVMessageTPCANMsg *msg);

public:
  int index;
  int subIndex;
protected:
  int canOpenObjectID;
private:
  // .....................................
  // .....................................
  string name;
  enum typeEnum type;
  enum accessEnum access;
  int rowInTable;
  // .....................................
  // .....................................
  union valType{
    unsigned int valU32;
    int val32;
  } val;
  // .....................................
  // .....................................
  string strValue;
  // .....................................
  // .....................................
  BVMessageTPCANMsg msg;
  // .....................................
  // .....................................
public:
  int pc2EposRead            ;
  int pc2EposWrite1Byte      ;
  int pc2EposWrite2Byte      ;
  int pc2EposWrite4Byte      ;
  int pc2EposWriteNotDefined ;

  int epos2PC1Byte  ;
  int epos2PC2Byte  ;
  int epos2PC4Byte  ;
  int epos2PCAck   ;
};


#endif

