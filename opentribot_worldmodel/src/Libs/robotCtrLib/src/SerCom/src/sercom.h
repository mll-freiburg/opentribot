#include <stdio.h>
#include <termios.h>
#include <sys/time.h>
#include <iostream>
#include "sercom_i.h"


class SerCom 
{
 private:
  std::ostream *errout, *infoout;
  
  SerComI *com_obj;

  // params to read and init
  int     com_mode;     static const int com_mode_def   =  ::SerComI::COMMODE_LINUX;
  
  void  init_params();
  bool  read_params           (const char   *name,                 // Name for parameter file to read from
			       const char   *chapter);              // chapter in parameterfile

 public:
 
 
  /*---CONSTRUCTORS--------------------------*/
  SerCom                     (const char   *_conf_fname,                 // Name for parameter file to read from
			      const char   *_conf_chapter,              // chapter in parameterfile
			      std::ostream* _errout=&std::cerr,        // ostream to write error messages
			      std::ostream* _infoout=&std::cout);      // ostream to write informations

			      
  /*---DESTRUCTOR----------------------------*/
  ~SerCom                    ();


  /*---PUBLIC FUNCTIONS----------------------*/
  void print_params           (std::ostream &_out);
  
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
