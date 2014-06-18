#ifndef _SERCOM_STD_H_
#define _SERCOM_STD_H_

#include "sercom_i.h"
#include <termios.h>

#define MAX_ASYNC_BUFFER 200

//#define DEBUG
//#define DEBUG_LEVEL   3

class SerComStd : public SerComI
{
 protected:  
  fd_set readfs;
  int    maxfd;     /* maximum file desciptor used */
  struct timeval ComTimeout;

  char string_buffer_async[MAX_ASYNC_BUFFER];
  bool string_buffer_async_empty;

  int fd;
  struct termios oldtio,newtio;


 public:
  SerComStd(const char * _conf_fname, const char * _conf_chapter, 
	    std::ostream* _errout=&std::cerr, std::ostream* _infoout=&std::cout) : SerComI( _conf_fname, _conf_chapter,_errout, _infoout){;};


  bool  init();
  bool  deinit();

  int flush_line(bool _verbose = 1);

  bool  talk                  (char    *_send, 
			       char    *_receive, 
			       int      _recv_len, 
			       int      _test = 0,
			       timeval *_sendtime=0, 
			       timeval *_receivetime=0,
			       bool     _verbose = 1);
  
  
  bool  readline              (char    *_receive,
			       int      _recv_len,
			       timeval *_receivetime=0,
			       bool     _verbose = 1);


  bool  readline_async        (char    *_receive,
			       int      _recv_len,
			       long     timeout_us,
			       timeval *_receivetime=0,
			       bool     _verbose = 1);

  bool  send                  (char    *_send,
			       timeval *_sendtime=0,
			       bool     _verbose = 1);  
  
};

#endif
