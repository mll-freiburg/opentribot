#include "sercom_i.h"
#include "ValueReader.h"
#include "my_err_msg.h"

SerComI::SerComI(const char * _conf_fname, const char * _conf_chapter, 
		 std::ostream* _errout, std::ostream* _infoout)
{
  errout   = _errout;
  infoout  = _infoout;

  init_params();

  read_params(_conf_fname, _conf_chapter);

  print_params(*infoout);

}

SerComI::~SerComI()
{
  THIS_INFOOUT("OK, bye!");
  (*errout) << std::flush;
  (*infoout) << std::flush;
  //delete errout;
  //delete infoout;
}

void SerComI::init_params()
{
  ser_mode    = ser_mode_def;
  ser_port    = ser_port_def;
  for (int i=0; i<10; i++) ser_speeds[i]=0;
  ser_speeds[0]=6500;
  ser_speeds[1]=38400;
  ser_speeds[2]=57600;
  ser_speed   = ser_speed_def;
  ser_stbits  = ser_stbits_def;
  inter_char_timeout  = inter_char_timeout_def;
  comm_timeout        = comm_timeout_def;
  num_junk_after_nl   = num_junk_after_nl_def;
}

bool SerComI::read_params(const char *_fname, const char * _chapter)
{
  ValueReader vr;
  
  if (!vr.append_from_file (_fname, _chapter))
    {
      THIS_ERROUT("Failed to read params for serial communication from file: " << _fname << " chapter: " << _chapter << " using defaults!.");
      return false;
    }
  if (!vr.get("SER_MODE",ser_mode))   THIS_ERROUT("Cant read param: SER_MODE, using default: " << ser_mode_def);
  if (!vr.get("SER_PORT",ser_port))   THIS_ERROUT("Cant read param: SER_PORT, using default: " << ser_port_def);
  if (vr.get("SER_SPEEDS",ser_speeds,10)<1) THIS_ERROUT("Cant read param: SER_SPEEDS, using defaults");
  if (!vr.get("COM_SPEED",ser_speed)) THIS_ERROUT("Cant read param: SER_SPEED, using default: " << ser_speed_def);
  if (!vr.get("STOPBITS",ser_stbits))  THIS_ERROUT("Cant read param: STOPBITS, using default: " << ser_stbits_def);
  if (!vr.get("INTER_CHAR_TIMEOUT",inter_char_timeout))  THIS_ERROUT("Cant read param: INTER_CHAR_TIMEOUT, using default: " << inter_char_timeout_def);
  if (!vr.get("COM_TIMEOUT",comm_timeout)) THIS_ERROUT("Cant read param: COMTIMEOUT, using default: " << comm_timeout_def);
  if (!vr.get("JUNK_AFTER_NL", num_junk_after_nl)) THIS_ERROUT("Cant read param: JUNK_AFTER_NL, using default: " << num_junk_after_nl_def);
  return true;
}

void SerComI::print_params(std::ostream & _out)
{
  _out << "SER_MODE"   << "  " << ser_mode << "\n"
       << "SER_PORT"   <<  "  " << ser_port << "\n"
       << "COM_SPEED"  <<  "  " << ser_speeds[ser_speed] << "\n"
       << "STOPBITS"   <<  "  " << ser_stbits << "\n"
       << "INTER_CHAR_TIMEOUT" <<  "  " << inter_char_timeout << "\n"
       << "COM_TIMEOUT"<<  "  " << comm_timeout << "\n"
       << "JUNK_AFTER_NL" << " " << num_junk_after_nl << "\n";
}

int   SerComI::flush_line(bool _verbose) {THIS_INFOOUT("This is only a virtual function!"); return 0;}
bool  SerComI::init()       {THIS_INFOOUT("This is only a virtual function!"); return true;}  
bool  SerComI::deinit()     {THIS_INFOOUT("This is only a virtual function!"); return true;}
bool  SerComI::talk                   (char    *_send, 
				       char    *_receive, 
				       int      _recv_len, 
				       int      _test,
				       timeval *_sendtime, 
				       timeval *_receivetime,
				       bool     _verbose)   {THIS_INFOOUT("This is only a virtual function!"); return true; }
bool  SerComI::readline               (char    *_receive,
				       int      _recv_len,
				       timeval *_receivetime,
				       bool     _verbose) {THIS_INFOOUT("This is only a virtual function!"); return true; }
bool  SerComI::readline_async         (char    *_receive,
				       int      _recv_len,
				       long     timeout_us,
				       timeval *_receivetime,
				       bool     _verbose) {THIS_INFOOUT("This is only a virtual function!"); return true; }
bool  SerComI::send                   (char    *_send,
				       timeval *_sendtime,
				       bool     _verbose) {THIS_INFOOUT("This is only a virtual function!"); return true; }
