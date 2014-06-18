#include "sercom_std.h"
#include <fcntl.h>
#include "my_err_msg.h"
#include <string.h>

#define TDIFF_US(__tv1__, __tv2__) (long)(((__tv2__.tv_sec-__tv1__.tv_sec)*1000+(__tv2__.tv_usec-__tv1__.tv_usec)/1000.0) * 1000)

bool SerComStd::init()
{
  char devicename[30];
  int _speed;

  switch (ser_mode) {
  case (::SerComI::COMMODE_LINUX) :
    sprintf(devicename,"/dev/ttyS%d",ser_port);
    break;
  case (::SerComI::COMMODE_LINUX_USB) :
    sprintf(devicename,"/dev/ttyUSB%d",ser_port);
    break;
  default:
    THIS_ERROUT("SER_MODE: " << ser_mode << " not recognized at this point!");
    return false;
  }
  
  fd = open(devicename, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd <0) {
    THIS_ERROUT("Error on opening " << devicename);
    return false;
  }
   
  THIS_INFOOUT("Success on opening " << devicename);
   
  maxfd = fd + 1;  /* maximum bit entry (fd) to test for select*/
 
  tcgetattr(fd,&oldtio); /* save current port settings */
  
  bzero(&newtio, sizeof(newtio));

  switch (ser_speeds[ser_speed]) {
  case 9600:
    _speed=B9600;
    break;
  case 19200:
    _speed=B19200;
    break;
  case 38400:
    _speed=B38400;
    break;
  case 57600:
    _speed=B57600;
    break;
  case 115200:
    _speed=B115200;
    break;
  default:
    THIS_ERROUT("Failed to set ser_speed " << ser_speed << " in this mode!!");
    return false;
  }
  
  switch (ser_stbits) 
    {
    case 1: newtio.c_cflag = _speed | CS8 | CLOCAL | CREAD;
      break;
    case 2: newtio.c_cflag = _speed | CS8 | CSTOPB | CLOCAL | CREAD;
      break;
    default:
      THIS_ERROUT("Stopbits " << ser_stbits << " only 1 and 2 stopbits are allowed in this mode!! (set to 2)");
      newtio.c_cflag = _speed | CS8 | CSTOPB;
    }
  
  newtio.c_iflag = 0;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;
  
  // newtio.c_cc[VTIME]    = 1;   /* inter-character timer*/
  int ic_timeout= inter_char_timeout/100;
  
  if (ic_timeout<1) 
    ic_timeout=0;
  newtio.c_cc[VTIME]    = ic_timeout;
  
  newtio.c_cc[VMIN]     = 0;   /* no blocking read*/
  
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);
  
  flush_line();

  string_buffer_async[0]='\0';
  string_buffer_async_empty=true;

  THIS_INFOOUT("OK, have fun :-)");
  return true;
}


bool SerComStd::deinit()
{
  tcsetattr(fd,TCSANOW,&oldtio);
  THIS_INFOOUT("deinited!!");

  return true;
}

int SerComStd::flush_line(bool _verbose)
{
  char c;
  int res;
  int nflush=-1;
  do {
    res=read(fd,&c,1);
    nflush++;
  } while(res>0);
  if (_verbose) THIS_INFOOUT("Flushing " << nflush << " bytes.");
  return nflush;
}


bool  SerComStd::talk (char    *_send, 
		       char    *_receive, 
		       int      _recv_len, 
		       int      _test,
		       timeval *_sendtime, 
		       timeval *_receivetime,
		       bool     _verbose)
{
  
  bool res=true;

  send(_send, _sendtime, _verbose);

  if (!readline(_receive, _recv_len, _receivetime, _verbose)) return false;
#ifdef DEBUG
  if (_verbose) THIS_INFOOUT("talk protocol -->[" << _send << "] <--[" << _receive << "]");
#endif
  if (_test == ::SerComI::T_NOTHING) res = true;
  else if( (_test == ::SerComI::T_SECOND) && !(_send[1]==_receive[0]) ) res = false;
  else if( (_test == ::SerComI::T_OK) && !((_receive[0]=='O')&&(_receive[1]=='K'))) res = false;
  else if( (_test == ::SerComI::T_FIRST_SMALL) && !(_receive[0]==tolower(_send[0]))) res = false;

  if (!res) 
    {
      if (_verbose) THIS_ERROUT("protocol error -->[" << _send << "] <--[" << _receive << "] test: " << _test);
      flush_line(_verbose);
      return false;
    }
  else 
    {
      int rsize;
      if ((rsize = strlen (_receive)) >= 2)	/* Strip terminating '\t\r\n' if any */
	{
	  if (_receive[rsize - 1] == '\n') _receive[rsize - 1] = '\0';
	  if (_receive[rsize - 1] == '\r') _receive[rsize - 1] = '\0';
	  if (_receive[rsize - 1] == '\t') _receive[rsize - 1] = '\0';
	  if (_receive[rsize - 2] == '\t') _receive[rsize - 2] = '\0';
	  if (_receive[rsize - 2] == '\r') _receive[rsize - 2] = '\0';
	}
    }
  
  return true;
}


bool SerComStd:: readline(char     *_receive,
			  int       _recv_len,
			  timeval  *_receivetime,
			  bool      _verbose)
{
  char c;
  int res;
  int rsize=0;
  int isfirst=1;

  ComTimeout.tv_usec = (comm_timeout%1000)*1000;  /* milliseconds to us */
  ComTimeout.tv_sec  = comm_timeout/1000;         /* seconds */
  
  FD_ZERO(&readfs);
  FD_SET(fd, &readfs);  /* set testing for source 1 */
  int select_res=select(maxfd, &readfs, NULL, NULL, &ComTimeout);
  if (select_res==0)
    {
      if (_verbose) THIS_ERROUT("Timeout. No response! Device connected?");
      return false;
    }
  if (FD_ISSET(fd,&readfs))
    {
      timeval time_act;
      timeval lcr;
      bool ic_timeout = false;
      gettimeofday(&time_act,0);
      do {
	c='\0';
	res=read(fd,&c,1);
	if (res>0) {
	  if(!(isfirst==1 && (c=='\t' || c==' '))) // ignore empty space at beginning of line (or left in buffer)
	    { 
	      _receive[rsize] = c;
	      rsize += res;
	      isfirst=-1;
	      gettimeofday(&lcr,0);
	    }
	}
	gettimeofday(&time_act,0);
	
	if ((isfirst!=1) && (TDIFF_US(lcr, time_act) > inter_char_timeout*1000)) ic_timeout = true;
	
      } while((c != '\n') && (rsize < _recv_len-1) && !ic_timeout);

      _receive[rsize]='\0';
      
      //read junk after answer
      for (int i=0; i<num_junk_after_nl; i++)
	read(fd,&c,1);

      if (rsize >= _recv_len-1) {
	if (_verbose) THIS_ERROUT("Overflow for receive buffer[" << _recv_len << "]  ... " << rsize);
      }

      if (_receivetime!=0) gettimeofday(_receivetime,0);

      /* remove the NL at the end */
      if (rsize>0 && _receive[rsize-1]=='\n') _receive[rsize-1]='\0';
      
      if (ic_timeout) {
	if (_verbose) THIS_ERROUT("Inter character timeout without receiving an NL!");
	flush_line();
	return false;
      }
    }
  else
    {
      if (_verbose) THIS_ERROUT("Got an strange select signal.");
      return false;
    }
#ifdef DEBUG
  if (_verbose) THIS_INFOOUT("Receiving: ["<< _receive <<"]");
#endif
  return true;
}


bool SerComStd:: readline_async(char     *_receive,
				int       _recv_len,
				long      timeout_us,
				timeval  *_receivetime,
				bool      _verbose)
{
  char c;
  int res;
  int rsize=0;
  int isfirst=1;

  timeval time_started_at;
  timeval time_act;
  timeval lcr;
  
  char * buf_ptr;

  if (!string_buffer_async_empty) {
    buf_ptr=&string_buffer_async[strlen(string_buffer_async)];
    isfirst=-1;
  }
  else buf_ptr = _receive;
  
  gettimeofday(&time_started_at,0);
  gettimeofday(&time_act,0);

  bool ic_timeout = false;
  bool c_timeout  = false;
  do {
    c='\0';
    res=read(fd,&c,1);
    if (res>0) {
      if(!(isfirst==1 && (c=='\t' || c==' '))) // ignore empty space at beginning of line (or left in buffer)
	{
#ifdef DEBUG
	  if (DEBUG_LEVEL > 2) THIS_ERROUT("Reading [" << c << "]");
#endif
	  buf_ptr[rsize] = c;
	  rsize += res;
	  isfirst=-1;
	  gettimeofday(&lcr,0);
	}
    }
    gettimeofday(&time_act,0);
    
    if ((isfirst!=1) && (TDIFF_US(lcr, time_act) > timeout_us)) ic_timeout = true;
    if ((isfirst==1) && (TDIFF_US(time_started_at, time_act) > timeout_us))  c_timeout  = true;
    
  } while( (c != '\n') && (rsize < _recv_len-1) 
	   && !ic_timeout && !c_timeout );
  
  buf_ptr[rsize]='\0';
  
  if (c_timeout) return false;

  if (ic_timeout)
    {
      if (string_buffer_async_empty)
	{
	  if (_verbose) THIS_INFOOUT("WARNING: Inter character timeout without receiving an NL ... buffering! \n May be the cycle time is too high.");
	  sprintf(string_buffer_async,"%s",_receive);
	  string_buffer_async_empty=false;
	}
      return false;
    }

  if (!string_buffer_async_empty)
    {
      sprintf(_receive,"%s",string_buffer_async);
      // THIS_ERROUT("Completed a buffer: " << _receive);
      string_buffer_async[0]='\0';
      string_buffer_async_empty=true;
    }
  

  //read junk after answer
  for (int i=0; i<num_junk_after_nl; i++)
    read(fd,&c,1);

  if (rsize >= _recv_len-1) if (_verbose) THIS_ERROUT("Overflow for receive buffer[" << _recv_len << "]  ... " << rsize);

 
  if (_receivetime!=0) gettimeofday(_receivetime,0);
  
  /* remove the NL at the end */
  if (rsize>0 && _receive[rsize-1]=='\n') _receive[rsize-1]='\0';
  
  
  
#ifdef DEBUG
  if (_verbose) THIS_INFOOUT("Receiving: ["<< _receive <<"]");
#endif
  return true;
}



bool  SerComStd::send (char    *_send,
		       timeval *_sendtime,
		       bool     _verbose)
{
  write(fd,_send,strlen(_send));
#ifdef DEBUG
  if (_verbose) THIS_INFOOUT("Sending: [" << _send << "]");
#endif
  if (_sendtime!=0) gettimeofday(_sendtime,0);
  return true;
}
