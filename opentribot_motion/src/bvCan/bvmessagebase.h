#ifndef BV_MESSAGEBASE
#define BV_MESSAGEBASE

#include "bvbinaryio.h"
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

/** this is a base class for all message classes */
struct BVMessageBase{

public:
  virtual ~BVMessageBase(){};		
protected:
  /** these two functions should be implemented in the derived classes */	
  virtual void readCostum( istream* astream ) = 0 ;
  virtual void writeCostum( ostream* astream ) = 0 ;
public:
  void read(istream* astream);
  void write(ostream* astream);

public:

  /**
    this function wrap all the data members in the form of a string.
    HOW :
      - the function will pass a stringstream object reference to the
        virtual void write(ostream* astream).
      - virtual void write(ostream* astream), will write the data members to the 
        stringstream.
      @return, the string which is wrapped in the stringstream	
  */
  string objectToString();
 
  /** 
     this function will be used in order to initialize the object, 
     with a string.
     the string contains all the necessary data members, needed by the object.

     @param, a string which contains all the data needed for the initialization 
     of the object
  */ 
  // stringToObject
  void stringToObject( string anOjectInStringFormat );

  /** @return the object ID */
  int getID();
  int getSender( ){ return senderID; }
  int getReciever( ){ return receiverID; }
  long getTime(){ return time;}
  int getFlag(){ return flag; }

  void setSender( int senderID );
  void setReceiver( int ReceiverID );
  void setTime( long atime );
  void setFlag( int aflag );
public:
  static int getID( string *anOjectInStringFormat);
  static int getSenderID( string *anOjectInStringFormat);
  static int getRecieverID( string *anOjectInStringFormat);
  static long getTime( string *anOjectInStringFormat );
  static int getFlag( string *anOjectInStringFormat);

  static void setSenderID( string *anOjectInStringFormat , int aSenderID);
  static void setRecieverID( string *anOjectInStringFormat , int aReceiverID);
  static void setTime( string *anOjectInStringFormat , long atime );
  static void setFlag( string *anOjectInStringFormat , int aflag );
protected:
  /** object ID */
  int id;
  int senderID;
  int receiverID; 
  long time;
  int flag; 
};
#endif
