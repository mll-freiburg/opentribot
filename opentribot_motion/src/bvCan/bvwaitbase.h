#ifndef BV_WAIT_BASE
#define BV_WAIT_BASE
using namespace std;

class BVWaitBase {
public:
   virtual ~BVWaitBase(){}
public:
  virtual bool wait( int param , double timeout=0 ) = 0;
  virtual int trywait( int param ) = 0;
  virtual int read(void* param)=0;
  virtual int write(void* param)=0;

  void setrfd( int arfd ){ rfd=arfd;}
  int getrfd() const { return rfd;}  
  void setwfd( int awfd ){ wfd=awfd;}
  int getwfd() const { return wfd;}  
  
private:
  int rfd;  
  int wfd;  
};

#endif


