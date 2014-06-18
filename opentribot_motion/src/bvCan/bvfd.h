#ifndef BVFD
#define BVFD

#include "bvdebug.h"
#include "bvbinaryio.h"

#include "bvwaitbase.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>    // bzero
#include <string>
#include <sstream> 
#include "sys/ioctl.h"
#include <sys/time.h>
#include <errno.h>
using namespace std;

/**
    BVfd is a general class for handling a Linux file descriptor
    
    A file descriptor is simply an integer.
    some functions which use a file descriptor are:
    
      - ssize_t read(int fd, void *buf, size_t count);
      - ssize_t write(int fd, const void *buf, size_t count);
      - int select(int n, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout);
      - int fcntl(int fd, int cmd);
      
    this class will wrap a file descriptor and the above mentioned functionalities  
    the member functions of this class do the followings:
      - read from file descriptor
      - write to file descriptor
      - toggle between blocking and nonblocking modes
      - in nonblocking mode, I will use wait function before each IO operation,
        in order to know the status of the file descriptor

    this class is designed to be a base class for another classes that needs to
    use a file descriptor, like:
      BVSerial, for serial communications
      BVSocket, for socket communications
    
    so, in this class I will not assign a specific valid value to the file descriptor.
    A valid value should be assigned to the file descriptor in derived classes, in ctor 
    of them
*/

class BVfd : public BVWaitBase {

public:
 enum function   { readable = 0, writable = 1 };
public:
  /** ctor  */		       
  BVfd();
  /** cotr */
  BVfd(int afd);
  /** dtor */
  ~BVfd();
protected:
  /** set the file descriptor */
  void setfd(int afd);
public:
  /** get the file descriptor */
  int getfd() const; 

public:
  /** interface : of BVWaitBase */
  virtual bool wait( int param , double max_wait=0) ;
  virtual int trywait( int param ) ;
  virtual int read(void* param) ;
  virtual int write(void* param); 
   
public:
  /** @return true if mode is blocking, false otherwise */
  int is_blocking() ;

  /** toggle_blocking() could be const, but conceptionally it isn't */
  void toggle_blocking()/*const*/;
public:
  /** read "count" bytes from the file descriptor
      this function calls
        ssize_t read(int fd, void *buf, size_t count);
      it will handle the case when the fd is a non valid discriptor.
      
      @return , the return value of the 
        ssize_t read(int fd, void *buf, size_t count);
  */
  ssize_t read( void *buffer, size_t count);

  /** write count bytes to the file descriptor
      this function calls
        ssize_t ::write(int fd, const void *buf, size_t count);
      it will handle the case when the fd is a non valid discriptor.

      @return , the return value of the
        ssize_t write(int fd, const void *buf, size_t count);
  */
  ssize_t write(const void *buffer, size_t count);

protected: 
  /** write to log device a string based on errno
      this function will be called by read and write when the
      return value of ::read and ::write is less than zero
  */
  void errorLog(int errnoold) ;    
};

#endif
