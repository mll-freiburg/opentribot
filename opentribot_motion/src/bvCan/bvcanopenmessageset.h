#ifndef BV_CAN_OPEN_MESSAGE_SET
#define BV_CAN_OPEN_MESSAGE_SET

#include "bvcanopenmessage.h"
#include "bvcanopenobject.h"
using namespace std;

class BVCanOpenMessageSet{
public:
  BVCanOpenMessageSet();
  ~BVCanOpenMessageSet();
public:
  int size();

  int getCanOpenObjectID( BVMessageTPCANMsg *msg );
  bool getNameOfObject(int ainIndex , string* aname);
public:
  BVCanOpenObject *objs[100];
private:
  int vecSize;

};


#endif 
