#include "bvmessagebase.h"


/** returns the ID of the sender in the anOjectInStringFormat
*/
int BVMessageBase::getSenderID( string *anOjectInStringFormat){
  return (int)((anOjectInStringFormat->c_str())[0]);
}
void BVMessageBase::setSenderID( string *anOjectInStringFormat , int aSenderID){
  int* ptr = (int*)(anOjectInStringFormat->c_str()+0);
  *ptr = aSenderID;
}

int BVMessageBase::getRecieverID( string *anOjectInStringFormat ){
  return (int)((anOjectInStringFormat->c_str())[4]);
}
void BVMessageBase::setRecieverID( string *anOjectInStringFormat , int aReceiverID){
  int* ptr = (int*)(anOjectInStringFormat->c_str()+4);
  *ptr = aReceiverID;
}

/** returns the ID of hte anOjectInStringFormat
*/
int BVMessageBase::getID( string *anOjectInStringFormat){
  return (int)((anOjectInStringFormat->c_str())[8]);
}

long BVMessageBase::getTime( string *anOjectInStringFormat){
  return (long)((anOjectInStringFormat->c_str())[12]);
}
int BVMessageBase::getFlag( string *anOjectInStringFormat){
  return (int)((anOjectInStringFormat->c_str())[16]);
}
void BVMessageBase::setFlag( string *anOjectInStringFormat , int aflag){
  int* ptr = (int*)(anOjectInStringFormat->c_str()+16);
  *ptr = aflag;

}

void BVMessageBase::setTime( string *anOjectInStringFormat , long atime ){
  long* ptr = (long*)(anOjectInStringFormat->c_str()+12);
  *ptr = atime;
}



void BVMessageBase::read(istream* astream){
  *astream > senderID > receiverID > id > time > flag;
  readCostum(astream); 

}
void BVMessageBase::write(ostream* astream){
  *astream < senderID < receiverID < id < time < flag;
  writeCostum(astream); 
}

/**
   this function wrap all the data members in the form of a string.
   HOW :
     - the function will pass a stringstream object reference to the
       virtual void write(ostream* astream).
     - virtual void write(ostream* astream), will write the data members to the
       stringstream.
   @return, the string which is wrapped in the stringstream
*/
string BVMessageBase::objectToString(){
  stringstream sstr;
  write(&sstr);
  return sstr.str();
}

/**
  this function will be used in order to initialize the object,
  with a string.
  the string contains all the necessary data members, needed by the object.
  @param, a string which contains all the data needed for the initialization
  of the object
*/
void BVMessageBase::stringToObject( string anOjectInStringFormat ){
	
  // create a stringstream object
  stringstream sstr;
  const char* buf;
  // put the input string byte by byte in the sstr
  buf = anOjectInStringFormat.c_str();
  for(unsigned int i=0 ; i < anOjectInStringFormat.size() ; i++)
    sstr < buf[i];
  // now the object will read all the data it needs, from
  // the stringstream
		    
  read(&sstr);
}

/** @return the object ID */
int BVMessageBase::getID(){ return id; }

void BVMessageBase::setSender( int aSenderID ){ senderID = aSenderID; }
void BVMessageBase::setReceiver( int aReceiverID ){ receiverID = aReceiverID; }
void BVMessageBase::setTime( long atime ){ time = atime; }
void BVMessageBase::setFlag( int aflag ){ flag = aflag; }
