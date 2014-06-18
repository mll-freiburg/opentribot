#include "Tmc200.h"
#include "ValueReader.h"
#include "my_err_msg.h"
#include <cmath>
#include <string.h>
#include <cstdlib>


#define  TMC_TALK( __test__ )  \
  if ( !(tmc_com_res = sercom->talk(tmc_command, tmc_answer, TMC_COM_BUF_SIZE - 1, __test__  , &tmc_send_time, &tmc_recv_time))) \
    { \
      THIS_ERROUT(" serial communication error: " << tmc_com_res); \
    } \

#define  TMC_SEND  \
  if ( !(tmc_com_res = sercom->send(tmc_command, &tmc_send_time)) )	\
    { \
      THIS_ERROUT(" serial communication error: " << tmc_com_res); \
    } \


Tmc200::Tmc200(const char* _conf_fname, std::ostream *_errout, std::ostream *_infoout)
{
  errout  = _errout;
  infoout = _infoout;

  sercom = NULL;  
  sercom = new SerCom( _conf_fname, "TMC200_SERCOM", errout, infoout);

  init_tmc_params();

  sprintf(config_fname,"%s",_conf_fname);

  read_tmc_params(config_fname, TMC200_CHAPTER );

}


Tmc200::~Tmc200()
{
  setVelocity(0,0,0);

  delete params.Drive[0];
  delete params.Drive[1];
  delete params.Drive[2];

  if (sercom != NULL) delete sercom;
  THIS_INFOOUT("ok, bye!");
  
  (*errout) << std::flush;
  (*infoout) << std::flush;
}


void Tmc200::init_tmc_params()
{
  params.tmc_version = -1;
  params.workingmode = -1;
  params.sendmode    = -1;
  params.Drive[0]    = NULL;
  params.Drive[1]    = NULL;
  params.Drive[2]    = NULL;
  params.vcc         = 24.0;
  
  lost_connection = true;

}

bool Tmc200::read_tmc_params(const char *_f_name, const char *_chapter)
{
  ValueReader vr;
  bool        paramsok=true;

  if (! vr.append_from_file (_f_name, _chapter))
    {
      THIS_ERROUT("Problem with reading " << _f_name << " in Chapter " << _chapter);
      paramsok = false;
    }
  
  if (vr.get ("WORKING_MODE", params.workingmode) < 0 ) 
    {
      THIS_ERROUT("Can't read param: WORKING_MODE, using default: "<< workingmode_default);
      params.workingmode=workingmode_default;
      paramsok = false;
    }

  if (vr.get ("SEND_MODE", params.sendmode) < 0 ) 
    {
      THIS_ERROUT("Can't read param: SEND_MODE, using default: "<< sendmode_default);
      params.sendmode=sendmode_default;
      paramsok = false;
    }
  
  if (vr.get ("TMC_VERSION", params.tmc_version) < 0 ) 
    {
      THIS_ERROUT("Can't read param: TMC_VERSION, using default: "<< tmc_version_default);
      params.tmc_version = tmc_version_default;
      paramsok = false;
    }

  // read some params for SW Version >= 1.16
  if (params.tmc_version >= 2)
    {
      if (vr.get ("CLIMIT_MODE", params.c_limit_mode) < 0 ) 
	{
	  THIS_ERROUT("Can't read param: CLIMIT_MODE, using default: "<< c_limit_mode_default);
	  params.c_limit_mode = c_limit_mode_default;
	  paramsok = false;
	}
      if (vr.get ("CMEASURE_MODE", params.c_measure_mode) < 0 ) 
	{
	  THIS_ERROUT("Can't read param: CMEASURE_MODE, using default: "<< c_measure_mode_default);
	  params.c_measure_mode = c_measure_mode_default;
	  paramsok = false;
	}
      if (vr.get ("TMPARAM_TW", params.tw) < 0 ) 
	{
	  THIS_ERROUT("Can't read param: TMPARAM_TW, using default: "<< tw_default);
	  params.tw = tw_default;
	  paramsok = false;
	}
      if (vr.get ("TMPARAM_KTN", params.ktn) < 0 ) 
	{
	  THIS_ERROUT("Can't read param: TMPARAM_KTN, using default: "<< ktn_default);
	  params.ktn = ktn_default;
	  paramsok = false;
	}
      if (vr.get ("TMPARAM_KTZ", params.ktz) < 0 ) 
	{
	  THIS_ERROUT("Can't read param: TMPARAM_KTZ, using default: "<< ktz_default);
	  params.ktz = ktz_default;
	  paramsok = false;
	}
      if (vr.get ("THYSPARAM_OUT", params.max_temp_out) < 0 ) 
	{
	  THIS_ERROUT("Can't read param: THYSPARAM_OUT, using default: "<< max_temp_out_default);
	  params.max_temp_out = max_temp_out_default;
	  paramsok = false;
	}
      if (vr.get ("THYSPARAM_IN", params.max_temp_in) < 0 ) 
	{
	  THIS_ERROUT("Can't read param: THYSPARAM_IN, using default: "<< max_temp_in_default);
	  params.max_temp_in = max_temp_in_default;
	  paramsok = false;
	}
    }

  for (int i=0; i<3; i++) if(params.Drive[i]!=NULL) delete params.Drive[i];

  if (paramsok)
    THIS_INFOOUT("OK, params all read sucessfully!");
  else
    THIS_ERROUT("Warning, at least some parameters have not been read from file, default values are used!");

  params.Drive[0]= new SingleDrive(_f_name, "DRIVE", errout,infoout);
  params.Drive[1]= new SingleDrive(_f_name, "DRIVE", errout,infoout);
  params.Drive[2]= new SingleDrive(_f_name, "DRIVE", errout,infoout);

  return true;
}


float Tmc200::get_max_motor_vel(int i)
{
  if (i<0 || i>2) return 0;
  int vmax,imp,dist;
  float gp;
  params.Drive[i]->get_motor_params(imp,vmax,dist);
  params.Drive[i]->get_gear_param(gp);
  return (((vmax/gp) * (2*M_PI)) / 60.0);
}

bool Tmc200::ping()
{
  for (int i=0; i<10; i++)
    {
      sprintf(tmc_command,"SMODE %d\n", 2);
      tmc_com_res = sercom->talk(tmc_command, tmc_answer, 
				 TMC_COM_BUF_SIZE - 1, SerComI::T_OK , 
				 &tmc_send_time, &tmc_recv_time, 
				 0);
      if (tmc_com_res) return true;
    }
  return false;
}

// init the board with params stored in struct params
bool Tmc200::init()
{
  bool res=true;
  lost_connection = false;
  cmd_counter = 0;
  loopcounter = 0;

  if (!ping())
    {
      THIS_ERROUT("Kann board nicht anpingen... Board angeschlossen und betriebsbereit?");
      lost_connection = true;
      return false;
    }

  if (!setWorkingMode2(params.workingmode)) res=false;

  if (!setSendMode2(params.sendmode)) res=false;

  int imp, vel, dist;
  for (int i=0; i<3; i++)
    {
      params.Drive[i]->get_motor_params(imp,vel,dist);
      if (!setEncParam(i+1, imp, vel, dist)) res=false;
    }
  int kp,ki,kd;
  for (int i=0; i<3; i++)
    {
      params.Drive[i]->get_pid_params(kp,ki,kd);
      if (!setPIDParam(i+1, kp, ki, kd)) res=false;
    }
  int cnom, cmax;
  for (int i=0; i<3; i++)
    {
      params.Drive[i]->get_current_params(cnom,cmax);
      if (!setCLimitParam2(i+1, cnom, cmax)) res=false;
    }
  
  if (!set_port1L(0)) res=false; // kicker aus
  
  // init params of new tmc firmware >= 1.16
  if (params.tmc_version >= 2)
    {
      if (!setCLimitMode(params.c_limit_mode)) res = false;
      if (!setCMeasureMode(params.c_measure_mode)) res = false;
      for (int i=0; i<3; i++)
	{
	  if (!setTModelParams2(i+1, params.tw, params.ktn, params.ktz)) res=false;
	  if (!setTHystParams2(i+1, params.max_temp_out, params.max_temp_in)) res= false;
	}
    }

  if (!res)
    {
      THIS_ERROUT("Initialisierung fehlgeschlagen ... Board angeschlossen und betriebsbereit?");
      lost_connection = true;
    }
  else
    {
      THIS_INFOOUT("OK, all parameters set, have fun!");
      lost_connection = false;
    }
  return res;
}

bool Tmc200::reinit()
{
  THIS_ERROUT("Lost connection to board ... try to reinit ...");
  usleep(50000);
  sercom->flush_line();

  while (!not_received_async_cmds.empty()) not_received_async_cmds.pop();

  cmd_counter = 0;

  bool res = false;
  if (ping()) {
    res = init();
    if (res) 
      {
	THIS_ERROUT("Success on reinit ...");
	return res;
      }
  }
  
  THIS_ERROUT("Failed to reinit ...");
  return false;
}

// *********** Working Mode Fkt. ********************************************************
// Funktionen Für alle Boards gleich ... Bedeutung der Arbeitsmodi Variiert!!!

bool Tmc200::setWorkingMode2(int _mode)
{
  if ((_mode<0) || (_mode>2))
    {
      THIS_ERROUT("Arbeitsmodus " << _mode << " nicht vorhanden!");
      return false;
    }
  return setWorkingMode(_mode);
}

bool Tmc200::setWorkingMode(int _mode)
{
  sprintf(tmc_command,"SMODE %d\n", _mode);
  TMC_TALK(SerComI::T_OK);  
  if (!tmc_com_res) return false;
  params.workingmode = _mode;
  return true;
}

bool Tmc200::setWorkingMode_async(int _mode)
{
  if(not_received_async_cmds.size() > 20)
    not_received_async_cmds.pop();
  if (  cmd_counter > 20)
    {
      lost_connection = true;
      THIS_ERROUT("MAXIMAL NUMBER OF ASYNC COMMANDS WITHOUT ANSWER REACHED!");
    }
  if (lost_connection)
    {
      reinit();
      return false;
    }

  sprintf(tmc_command,"SMODE %d\n", _mode);
  TMC_SEND;
  if (!tmc_com_res) return false;

  TLabeledData cmd;
  cmd.data = ASYNC_CMD_WM;
  cmd.tv.tv_sec  = tmc_send_time.tv_sec;
  cmd.tv.tv_usec = tmc_send_time.tv_usec;
  not_received_async_cmds.push(cmd);
  cmd_counter ++;
  loopcounter ++;
  params.workingmode = _mode;
  return true;
}

bool Tmc200::getWorkingMode(int &_mode)
{
  
  bool ret=getWorkingMode();
  _mode = params.workingmode;
  return ret;
}

bool Tmc200::getWorkingMode()
{
  int res;
  int m;

  sprintf(tmc_command,"GMODE \n");
  TMC_TALK(SerComI::T_SECOND);
  if (tmc_com_res)
    {
      res=sscanf(tmc_answer,"MODE %d", &m);
      if (res==1)
	params.workingmode = m;
      else
	{
	  THIS_ERROUT("Error on parsing answer string!");
	  return false;
	}
    }
  else
    return false;
  
  return true;
}
// END *********** Working Mode Fkt. ********************************************************



// *********** Send Mode Fkt. ********************************************************
bool Tmc200::setSendMode2(int _mode)
{
  // param check!!
  return setSendMode(_mode);
}

bool Tmc200::setSendMode(int _mode)
{

  sprintf(tmc_command,"SSEND %d\n", _mode);
  TMC_TALK(SerComI::T_OK);  

  if (tmc_com_res)
      params.sendmode = _mode;
  else
    return false;

  return true;
}

bool Tmc200::getSendMode(int &_mode)
{
  bool ret=getSendMode();
  _mode=params.sendmode;
  return ret;
}

bool Tmc200::getSendMode()  // TMCPROTOCOLERROR
{
  int res;
  int m;

  sprintf(tmc_command,"GSEND \n");
  TMC_TALK(SerComI::T_SECOND);
  if (tmc_com_res)
    {
      res=sscanf(tmc_answer,"SMODE %d", &m);
      if (res==1)
	params.sendmode = m;
      else
	{
	  THIS_ERROUT("Error on parsing answer string!");
	  return false;
	}
    }
  else
    return false;
  
  return true;
}
// END *********** Send Mode Fkt. ********************************************************



// *********** Encoder Param Fkt. ********************************************************
bool Tmc200::setEncParam2(int _motor, int _imp, int _max_v, int _dist)
{
  if (_motor < 1 || _motor > 3)
    {
      THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
      return false;
    }
  if (_imp <0 || _imp > 25000)
    {
      THIS_ERROUT("Parameter impuls number out of Tmc200 range: " << _imp);
      return false;
    }
  if (_max_v < 0 || _max_v > 20000)
    {
      THIS_ERROUT("Parameter max vel out of Tmc200 range: " << _max_v);
      return false;
    }
  if (_dist < 0 || _dist > 1000)
    {
      THIS_ERROUT("Parameter dist out of Tmc200 range: " << _dist);
      return false;
    }
  return setEncParam(_motor, _imp, _max_v, _dist);
}


bool Tmc200::setEncParam(int _motor, int _imp, int _max_v, int _dist)
{

  sprintf(tmc_command,"SENCO M%d %d %d %d\n", _motor, 2*_imp, _max_v, _dist);
  TMC_TALK(SerComI::T_OK);  

  if (!tmc_com_res) return false;
  
  params.Drive[_motor-1]->set_motor_params(_imp, _max_v, _dist);

  return true;
}

bool Tmc200::getEncParam2(int _motor, int &_imp, int &_max_v, int &_dist)
{
  if (_motor < 1 || _motor > 3)
    {
      THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
      return false;
    }

  return getEncParam(_motor, _imp, _max_v, _dist);

}

bool Tmc200::getEncParam(int _motor, int &_imp, int &_max_v, int &_dist)
{
  bool ret=getEncParam(_motor);
  params.Drive[_motor-1]->get_motor_params(_imp, _max_v, _dist);
  return ret;
}

bool Tmc200::getEncParam2(int _motor)
{
  if (_motor < 1 || _motor > 3)
    {
      THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
      return false;
    }
  return getEncParam(_motor);
}

bool Tmc200::getEncParam(int _motor)
{
  int res;
  int m,i,v,d;

  sprintf(tmc_command,"GENCO M%d\n", _motor);
  TMC_TALK(SerComI::T_SECOND);
  if (!tmc_com_res) return false;
  res=sscanf(tmc_answer,"ENCO M%d %d %d %d", &m, &i, &v, &d);
  if (res==4)
    params.Drive[_motor-1]->set_motor_params( (int)(i/2.) , v, d);
  else
    {
      THIS_ERROUT("Error on parsing answer string!");
      return false;
    }
  return true;
}
// END *********** Encoder Params ********************************************************



// *********** PID Param Fkt. ********************************************************

bool Tmc200::setPIDParam2(int _motor, int _kp, int _ki, int _kd)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  if (_ki<0 || _ki>1000) {
    THIS_ERROUT("Parameter ki out of Tmc200 range: " << _ki);
    return false;
  }
  if (_kd<0 || _kd>1000) {
    THIS_ERROUT("Parameter kd out of Tmc200 range: " << _kd);
    return false;
  }
  if (_kp<0 || _kp>1000) {
    THIS_ERROUT("Parameter kp out of Tmc200 range: " << _kp);
    return false;
  }
  return setPIDParam(_motor,_kp, _ki, _kd);
}


bool Tmc200::setPIDParam(int _motor, int _kp, int _ki, int _kd)
{
  sprintf(tmc_command,"SVREG M%d %d %d %d\n", _motor, _kp, _ki, _kd);
  TMC_TALK(SerComI::T_OK); 
  if (!tmc_com_res) return false;
  params.Drive[_motor-1]->set_pid_params(_kp, _ki, _kd);
  return true;
}

bool Tmc200::getPIDParam2(int _motor)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  return getPIDParam(_motor);
}

bool Tmc200::getPIDParam2(int _motor, int &_kp, int &_ki, int &_kd)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  return getPIDParam(_motor, _kp,_ki,_kd);
}

bool Tmc200::getPIDParam(int _motor)
{
  int m,p,i,d;
  sprintf(tmc_command,"GVREG M%d\n", _motor);
  TMC_TALK(SerComI::T_SECOND);
  if (!tmc_com_res) return false;
  int res=sscanf(tmc_answer,"VREG M%d %d %d %d", &m, &p, &i, &d);
  if (res==4)
    params.Drive[_motor-1]->set_pid_params(p, i, d);
  else
    {
      THIS_ERROUT("Error on parsing answer string!");
      return false;
    }
  return true;
}

bool Tmc200::getPIDParam(int _motor, int &_kp, int &_ki, int &_kd)
{
  params.Drive[_motor-1]->get_pid_params(_kp,_ki,_kd);
  return true;
}

// END *********** PID Param Fkt. ********************************************************


// *********** Current Param Fkt. ********************************************************

bool Tmc200::setCLimitParam2(int _motor, int _c_nom, int _c_max)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  if (_c_nom<0 || _c_nom>10000) {
    THIS_ERROUT("Parameter c_nom out of Tmc200 range: " << _c_nom);
    return false;
  }
  if (_c_max<0 || _c_max>10000) {
    THIS_ERROUT("Parameter c_max out of Tmc200 range: " << _c_max);
    return false;
  }
  return setCLimitParam(_motor,_c_nom, _c_max);
}


bool Tmc200::setCLimitParam(int _motor, int _c_nom, int _c_max)
{
  sprintf(tmc_command,"SCLIM M%d %d %d\n", _motor, (int)((_c_max/10000.0)*1024.0), (int)((_c_nom/10000.0)*1024.0) );
  TMC_TALK(SerComI::T_OK); 
  if (!tmc_com_res) return false;
  params.Drive[_motor-1]->set_current_params(_c_nom, _c_max);
  return true;
}


bool Tmc200::getCLimitParam2(int _motor)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  return getCLimitParam(_motor);
}

bool Tmc200::getCLimitParam2(int _motor, int &_c_nom, int &_c_max)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  return getCLimitParam(_motor, _c_nom,_c_max);
}

bool Tmc200::getCLimitParam(int _motor)
{
  int m,nom,max;
  sprintf(tmc_command,"GCLIM M%d\n", _motor);
  TMC_TALK(SerComI::T_SECOND);
  if (!tmc_com_res) return false;
  int res=sscanf(tmc_answer,"GCLIM M%d %d %d", &m, &max, &nom);
  if (res==3)
    params.Drive[_motor-1]->set_current_params((int) ((nom/1024.0) * 10000.0), (int) ((max/1024.0) * 10000.0));
  else
    {
      THIS_ERROUT("Error on parsing answer string!");
      return false;
    }
  return true;
}

bool Tmc200::getCLimitParam(int _motor, int &_c_nom, int &_c_max)
{
  params.Drive[_motor-1]->get_current_params(_c_nom,_c_max);
  return true;
}

// END *********** current Param Fkt. ********************************************************


// *********** ClimitMode Functions ***********************************************************
bool Tmc200::setCLimitMode(int _mode)
{
  if (params.tmc_version < 2) {
    THIS_ERROUT("You called a function not available in this tmc version!");
    return false;
  }
  sprintf(tmc_command,"SESCL %d \n", (_mode !=0 ? 1 : 0) );
  TMC_TALK(SerComI::T_OK);
  return tmc_com_res;
}
bool Tmc200::getCLimitMode()
{
  if (params.tmc_version < 2) {
    THIS_ERROUT("You called a function not available in this tmc version!");
    return false;
  }
  int m;
  sprintf(tmc_command,"GESCL \n");
  TMC_TALK(SerComI::T_SECOND);
  if (!tmc_com_res) return false;
  int res=sscanf(tmc_answer,"ESCL %d", &m);
  if (res==1)
    params.c_limit_mode = m;
  else
    {
      THIS_ERROUT("Error on parsing answer string!");
      return false;
    }
  return true;
}
bool Tmc200::getCLimitMode(int &_mode)
{
  _mode = params.c_limit_mode;
  return true;
}
// END ** CLimitMode Functions ******************************************************************

//BEGIN CMeasureMode ****************************************************************************
bool Tmc200::setCMeasureMode(int _mode)
{
 if (params.tmc_version < 2) {
    THIS_ERROUT("You called a function not available in this tmc version!");
    return false;
  }
  sprintf(tmc_command,"SCMS %d \n", (_mode !=0 ? 1 : 0) );
  TMC_TALK(SerComI::T_OK);
  return tmc_com_res;
}
//END ** CMeasureMode ***********************************************************************+***


//BEGIN Thermic Model Params Functions *********************************************************

bool Tmc200::setTModelParams2(int _motor, int _tw, int _ktn, int _ktz)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  if (_tw<1 || _tw>100) {
    THIS_ERROUT("Parameter tw out of Tmc200 range: " << _tw);
    return false;
  }
  if (_ktn<1 || _ktn>100) {
    THIS_ERROUT("Parameter ktn out of Tmc200 range: " << _ktn);
    return false;
  }
  if (_ktz<1 || _ktz>100) {
    THIS_ERROUT("Parameter ktz out of Tmc200 range: " << _ktz);
    return false;
  }
  return setTModelParams(_motor, _tw, _ktn, _ktz);
}
bool Tmc200::setTModelParams(int _motor, int _tw, int _ktn, int _ktz)
{
  sprintf(tmc_command,"STMCO M%d %d %d %d\n", _motor, _tw, _ktn, _ktz );
  TMC_TALK(SerComI::T_OK); 
  if (!tmc_com_res) return false;

  params.tw  = _tw;
  params.ktn = _ktn;
  params.ktz = _ktz;

  return true;
}
bool Tmc200::getTModelParams2(int _motor)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  return getTModelParams(_motor);
}
bool Tmc200::getTModelParams(int _motor)
{
  // parameter werden für jeden Motor als gleich angenommen
  sprintf(tmc_command,"GTMCO M%d\n", _motor);
  TMC_TALK(SerComI::T_SECOND);
  if (!tmc_com_res) return false;
  int _m,_tw, _ktn, _ktz;
  int res=sscanf(tmc_answer,"TMCO M%d %d %d %d", &_m, &_tw, &_ktn, &_ktz);
   if (res==4)
     {
       params.tw  = _tw;
       params.ktn = _ktn;
       params.ktz = _ktz;
     }
   else
     {
       THIS_ERROUT("Error on parsing answer string!");
       return false;
     }
   return true;
}
bool Tmc200::getTModelParams2(int _motor, int &_tw, int &_ktn, int &_ktz)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  return getTModelParams(_motor, _tw, _ktn, _ktz);
}
bool Tmc200::getTModelParams(int _motor, int &_tw, int &_ktn, int &_ktz)
{
  _tw  = params.tw;
  _ktn = params.ktn;
  _ktz = params.ktz;
  return true;
}
// END ** Thermic Model Params Functions *******************************************************

// BEGIN Thermic Hysterese params **************************************************************
bool Tmc200::setTHystParams2(int _motor, int _max_out, int _min_in)
{ 
  if (params.tmc_version < 2) {
    THIS_ERROUT("You called a function not available in this tmc version!");
    return false;
  }
   if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  if (_max_out<100 || _max_out>400) {
    THIS_ERROUT("Parameter _max_out out of Tmc200 range: " << _max_out);
    return false;
  }
  if (_min_in<1 || _min_in>100) {
    THIS_ERROUT("Parameter min_in out of Tmc200 range: " << _min_in);
    return false;
  }
  return setTHystParams(_motor, _max_out, _min_in);
}
bool Tmc200::setTHystParams(int _motor, int _max_out, int _min_in)
{ 
  if (params.tmc_version < 2) {
    THIS_ERROUT("You called a function not available in this tmc version!");
    return false;
  }
  sprintf(tmc_command,"STSCO M%d %d %d\n", _motor, _max_out, _min_in);
  TMC_TALK(SerComI::T_OK); 
  if (!tmc_com_res) return false;
  params.max_temp_out = _max_out;
  params.max_temp_in  = _min_in;
  return true;
}
bool Tmc200::getTHystParams2(int _motor)
{
  if (params.tmc_version < 2) {
    THIS_ERROUT("You called a function not available in this tmc version!");
    return false;
  }
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  return getTHystParams(_motor);
}
bool Tmc200::getTHystParams(int _motor)
{
  if (params.tmc_version < 2) {
    THIS_ERROUT("You called a function not available in this tmc version!");
    return false;
  }
  sprintf(tmc_command,"GTSCO M%d\n", _motor);
  TMC_TALK(SerComI::T_SECOND);
  if (!tmc_com_res) return false;
  int m, max_out, min_in;
  int res=sscanf(tmc_answer,"TSCO M%d %d %d", &m, &max_out, &min_in);
  if (res==3)
    {
      params.max_temp_out = max_out;
      params.max_temp_in  = min_in;
    }
  else
    {
      THIS_ERROUT("Error on parsing answer string!");
      return false;
    }
  return true;
}
bool Tmc200::getTHystParams2(int _motor, int &_max_out, int &_min_in)
{
  if (_motor<0 || _motor>3) {
    THIS_ERROUT("Parameter motor number out of Tmc200 range: " << _motor);
    return false;
  }
  return getTHystParams(_motor, _max_out, _min_in);
}
bool Tmc200::getTHystParams(int _motor, int &_max_out, int &_min_in)
{
  _max_out = params.max_temp_out;
  _min_in  = params.max_temp_in;
  return true;
}
// END ** Thermic Hysterese params *************************************************************


// POWER SUPLY FUNCTION *********************************************************************
bool Tmc200::getPowerSupplyVoltage(float &_v)
{
  int v;
  sprintf(tmc_command,"GADC UB \n");
  TMC_TALK(SerComI::T_SECOND); 
  if (!tmc_com_res) return false;
  int res=sscanf(tmc_answer,"ADC UB %d",&v);
  if (res==1)
    _v=v/23.0 + 1.5;
  else
    {
      THIS_ERROUT("Error on parsing answer string!");
      return false;
    }
  return true;
}
// END ** POWER SUPLY FUNCTION ***************************************************************

// TEMPERATURE FUNCTION *********************************************************************
bool Tmc200::getMotorTemp(int &t1, int &t2, int &t3)
{
  sprintf(tmc_command,"GWTEM \n");
  TMC_TALK(SerComI::T_SECOND); 
  if (!tmc_com_res) return false;
  int T1, T2, T3;
  int res=sscanf(tmc_answer,"WTEM %d %d %d",&T1, &T2, &T3);
  if (res!=3)
    {
      THIS_ERROUT("Error on parsing answer string!");
      return false;
    }
  t1=T1;
  t2=T2;
  t3=T3;
  return true;
}
// END ** TEMPERATURE FUNCTION ***************************************************************


// RESET FUNCTION ***************************************************************************
bool Tmc200::rebootTMC()
{
  if (params.tmc_version < 2) {
    THIS_ERROUT("You called a function not available in this tmc version!");
    return false;
  }
  sprintf(tmc_command,"SRESET TMC \n");
  TMC_TALK(SerComI::T_NOTHING);
  usleep(100000);
  sercom->flush_line(0);
  return true;
}
// END ** reset function ********************************************************************



// DIGITAL IO FUNCTIONS **********************************************************************
bool Tmc200::set_port1L(int _value, timeval *tstmp)
{
  if (lost_connection)
    {
      reinit();
      return false;
    }
  sprintf(tmc_command,"SP1L %d \n", _value);
  TMC_TALK(SerComI::T_OK);
  if (!tmc_com_res) return false;
  if (tstmp!=NULL)
    {
      tstmp->tv_sec=tmc_send_time.tv_sec;
      tstmp->tv_usec=tmc_send_time.tv_usec;
    }
  return true;
}

bool Tmc200::set_port1L_async(int _value, timeval *tstmp)
{
  if(not_received_async_cmds.size() > 20)
    not_received_async_cmds.pop();
  if (  cmd_counter > 20)
    {
      lost_connection = true;
      THIS_ERROUT("MAXIMAL NUMBER OF ASYNC COMMANDS WITHOUT ANSWER REACHED!");
    }
  if (lost_connection)
    {
      reinit();
      return false;
    }
  sprintf(tmc_command,"SP1L %d\n", _value);
  TMC_SEND;
  if (tmc_com_res) {
    TLabeledData cmd;
    cmd.data = ASYNC_CMD_PORT1L;
    cmd.tv.tv_sec  = tmc_send_time.tv_sec;
    cmd.tv.tv_usec = tmc_send_time.tv_usec;
    not_received_async_cmds.push(cmd);
    cmd_counter ++;
    loopcounter ++;
  }
  else return false;

  return true;
}
// END ** DIGITAL IO FUNCTIONS ***************************************************************




void Tmc200::print_params(std::ostream &_out)
{
  _out << " --- Params for Tmc200: --- \n"
       << "Tmc Version used: " << params.tmc_version << "\n"
       << "Workingmode: \n[" << WM_DO << "] direct output; \n[" << WM_DO_C << "] direct output with current limit;  \n[" << WM_PID_C << "] PID mode with current limit;\n"; 
  _out << "Act. workingmode: " << params.workingmode << "\n";

  _out << "SendMode: \n"
       << "Act. sendmode: " << params.sendmode << "\n";

  _out << "CLimitMode: \n[0: individual; 1: simultan]\n";
  if (params.tmc_version < 2)
    _out << "Not used in this Version of TMC\n";
  else
    _out << "Act. CLimitMode: " << params.c_limit_mode << "\n";

  _out << "Thermic Model Params: \n";
  if (params.tmc_version < 2)
    _out << "Not used in this Version of TMC\n";
  else
    _out << " TW: " << params.tw << " KTn: " << params.ktn << " KTz: " << params.ktz << "\n"; 

  _out << "Thermic Hysteresis Params: \n";
  if (params.tmc_version < 2)
    _out << "Not used in this Version of TMC\n";
  else
    _out << " OUT: " << params.max_temp_out << " IN: " << params.max_temp_in << "\n"; 

  for (int i=0; i<3; i++)
    {
      _out << "DRIVE " << i << "\n";
      params.Drive[i]->print(_out);
      _out << "\n";
    }
}




bool Tmc200::cmpt_radvel_to_tmcvel(float  _v1_rad, float  _v2_rad, float  _v3_rad,
				   int   &_v1_tmc, int   &_v2_tmc, int   &_v3_tmc)
{
  float v_wheel_rad[]={_v1_rad,_v2_rad,_v3_rad};
  int   v_motor_tmc[3];

  int vmax,imp,dist;
  float gp;
  bool res=true;

  for (int i=0; i<3; i++)
    {
      params.Drive[i]->get_motor_params(imp,vmax,dist);
      params.Drive[i]->get_gear_param(gp);
      v_motor_tmc[i]= (int)(((((v_wheel_rad[i] * 60.0)/ (2.0*M_PI) ) * gp) / (float)vmax) * 1024.0);
      if (abs(v_motor_tmc[i]) > 1024)
	{
	  THIS_ERROUT("Velocity too high for motor: " << v_wheel_rad[i] << "rad/s  bzw: " << v_motor_tmc[i] << " range (0-1024)");
	  if (v_motor_tmc[i] > 1024) v_motor_tmc[i]=1024; else v_motor_tmc[i]=-1024;
	  res=false;
	}
    }
  _v1_tmc=v_motor_tmc[0];
  _v2_tmc=v_motor_tmc[1];
  _v3_tmc=v_motor_tmc[2];
  return res;
}


bool Tmc200::cmpt_tmcvel_to_radvel(int    _v1_tmc, int    _v2_tmc, int    _v3_tmc,
				   float &_v1_rad, float &_v2_rad, float &_v3_rad)
{
  int   v_motor_tmc[]={_v1_tmc,_v2_tmc,_v3_tmc};
  float v_wheel_rad[3];

  int vmax,imp,dist;
  float gp;

  for (int i=0; i<3; i++)
    {
      params.Drive[i]->get_motor_params(imp,vmax,dist);
      params.Drive[i]->get_gear_param(gp);
      v_wheel_rad[i]=(((((v_motor_tmc[i] / 1024.0) * vmax) / gp) * (2*M_PI)) / 60.0);
    }

  _v1_rad = v_wheel_rad[0];
  _v2_rad = v_wheel_rad[1];
  _v3_rad = v_wheel_rad[2];

  return true;
}


bool Tmc200:: parse_answer_string(char * _answer, tmc_data_t &_data, timeval *tstmp)
{
  
  // new version also including TMC firmware >= 1.15
  
  char * strptr;
  int res;
  int p[3];
  
  // first have a look at the tmc_version
  if (params.tmc_version >= VERSION_116)
    {
      // things that are transfered in every cyclus (even in SMODE 0)
      // ignore ODO

      // scan for motor temperature
      strptr = strstr(tmc_answer, "TEM ");
      if (strptr==NULL)
	THIS_ERROUT("Failure on finding TEMP in answer string ( wrong tmc_version? )");
      else
	{
	  res=sscanf(strptr,"TEM %d %d %d", &p[0], &p[1], &p[2]);
	  if (res < 3) THIS_ERROUT("Error on parsing TEM values in: [" << strptr << "]");
	  else 
	    for (int i=0; i<3; i++) {
	      _data.T[i].data=p[i];
	      _data.T[i].tv.tv_sec  = tstmp->tv_sec;
	      _data.T[i].tv.tv_usec = tstmp->tv_usec;
	    }
	}
      
      // scan for temperature model flag
      strptr = strstr(tmc_answer, "SS ");
      if (strptr==NULL)
	THIS_ERROUT("Failure on finding SS in answer string (wrong tmc_version? )");
      else
	{
	  res=sscanf(strptr,"SS %d", &p[0]);
	  if (res < 1) THIS_ERROUT("Error on parsing SS values in: [" << strptr << "]");
	  else 
	    {
	      _data.SS.data=p[0];
	      _data.SS.tv.tv_sec  = tstmp->tv_sec;
	      _data.SS.tv.tv_usec = tstmp->tv_usec;
	    }
	}
      
      if (params.sendmode >= SM_P) // in the new firmware in mode 5 the PowerSupplyVoltage and a motorbridge flag is sent too
	{
	  // scan for temperature model flag
	  strptr = strstr(tmc_answer, "BE ");
	  if (strptr==NULL)
	    THIS_ERROUT("Failure on finding BE in answer string (wrong tmc_version? )");
	  else
	    {
	      res=sscanf(strptr,"BE %d", &p[0]);
	      if (res < 1) THIS_ERROUT("Error on parsing BE values in: [" << strptr << "]");
	      else 
		{
		  _data.BE.data=p[0];
		  _data.BE.tv.tv_sec  = tstmp->tv_sec;
		  _data.BE.tv.tv_usec = tstmp->tv_usec;
		}
	    }

	  // scan for power sypply voltage UB
	   strptr = strstr(tmc_answer, "UB ");
	  if (strptr==NULL)
	    THIS_ERROUT("Failure on finding UB in answer string (wrong tmc_version? )");
	  else
	    {
	      res=sscanf(strptr,"UB %d", &p[0]);
	      if (res < 1) THIS_ERROUT("Error on parsing UB values in: [" << strptr << "]");
	      else 
		{
		  _data.UB.data=p[0];
		  _data.UB.tv.tv_sec  = tstmp->tv_sec;
		  _data.UB.tv.tv_usec = tstmp->tv_usec;
		}
	    }
	  
	} 
    } // endif parse new params
  

  // parse old params
  res = sscanf (tmc_answer, "V %d %d %d \t C %d %d %d \t x %d \t dT %d \t D %d %d %d \t O %d %d %d", 
		&answ_params[0],  &answ_params[1],  &answ_params[2],
		&answ_params[3],  &answ_params[4],  &answ_params[5],
		&answ_params[6],  &answ_params[7], 
		&answ_params[8],  &answ_params[9],  &answ_params[10],
		&answ_params[11], &answ_params[12], &answ_params[13]);
  
  if (params.sendmode == SM_NOTHING) 
    {
      if (res==0) return true;
      else {
	THIS_ERROUT("Something wrong with number of received params! On parsing [" << _answer <<"]" << res );
	return false;
      }
    }  
  
  if (params.sendmode >= SM_V)
    {
      if (res>=3)
	{
	  _data.V[0].data        = -answ_params[0]; // negativ -> motor anschluss def.
	  _data.V[0].tv.tv_sec   = tstmp->tv_sec;
	  _data.V[0].tv.tv_usec  = tstmp->tv_usec;
	  _data.V[1].data        = -answ_params[1];
	  _data.V[1].tv.tv_sec   = tstmp->tv_sec;
	  _data.V[1].tv.tv_usec  = tstmp->tv_usec;
	  _data.V[2].data        = -answ_params[2];
	  _data.V[2].tv.tv_sec   = tstmp->tv_sec;
	  _data.V[2].tv.tv_usec  = tstmp->tv_usec;
	}
      else 
	{
	  THIS_ERROUT("Something wrong with number of received params! On parsing [" << _answer <<"]" << res  );
	  return false;
	}
      
      if (params.sendmode>=SM_C)
	{
	  if (res>=6)
	    {
	      _data.C[0].data       = answ_params[3];
	      _data.C[0].tv.tv_sec  = tstmp->tv_sec;
	      _data.C[0].tv.tv_usec = tstmp->tv_usec;
	      _data.C[1].data       = answ_params[4];
	      _data.C[1].tv.tv_sec  = tstmp->tv_sec;
	      _data.C[1].tv.tv_usec = tstmp->tv_usec;	      
	      _data.C[2].data       = answ_params[5];
	      _data.C[2].tv.tv_sec  = tstmp->tv_sec;
	      _data.C[2].tv.tv_usec = tstmp->tv_usec;
	    }
	  else 
	    {
	      THIS_ERROUT("Something wrong with number of received params! On parsing [" << _answer <<"]" << res );
	      return false;
	    }
	  
	  if (params.sendmode>=SM_N)
	    {
	      if (res>=8)
		{
		  _data.n.data       = answ_params[6];
		  _data.n.tv.tv_sec  = tstmp->tv_sec;
		  _data.n.tv.tv_usec = tstmp->tv_usec;
		  _data.dt.data      = answ_params[7];
		  _data.dt.tv.tv_sec = tstmp->tv_sec;
		  _data.dt.tv.tv_usec= tstmp->tv_usec;
		}
	      else 
		{
		  THIS_ERROUT("Something wrong with number of received params! On parsing [" << _answer <<"]" << res );
		  return false;
		}

	      if (params.sendmode >= SM_D)
		{
		  if (res>=11)
		    {
		      _data.D[0].data       = answ_params[8];
		      _data.D[0].tv.tv_sec  = tstmp->tv_sec;
		      _data.D[0].tv.tv_usec = tstmp->tv_usec;
		      _data.D[1].data       = answ_params[9];
		      _data.D[1].tv.tv_sec  = tstmp->tv_sec;
		      _data.D[1].tv.tv_usec = tstmp->tv_usec;
		      _data.D[2].data       = answ_params[10];
		      _data.D[2].tv.tv_sec  = tstmp->tv_sec;
		      _data.D[2].tv.tv_usec = tstmp->tv_usec;
		    }
		  else 
		    {
		      THIS_ERROUT("Something wrong with number of received params! On parsing [" << _answer <<"]" << res );
		      return false;
		    }

		  if (params.sendmode >= SM_P)
		    {
		      if (res==14)
			{
			  _data.O[0].data       = answ_params[11];
			  _data.O[0].tv.tv_sec  = tstmp->tv_sec;
			  _data.O[0].tv.tv_usec = tstmp->tv_usec;
			  _data.O[1].data       = answ_params[12];
			  _data.O[1].tv.tv_sec  = tstmp->tv_sec;
			  _data.O[1].tv.tv_usec = tstmp->tv_usec;
			  _data.O[2].data       = answ_params[13];
			  _data.O[2].tv.tv_sec  = tstmp->tv_sec;
			  _data.O[2].tv.tv_usec = tstmp->tv_usec;
			}
		      else 
			{
			  THIS_ERROUT("Something wrong with number of received params! On parsing [" << _answer <<"]" << res);
			  return false;
			}
		    } 
		}
	    }
	}
    }
  return true;
}

// berechne reale Werte aus tmc Werten
bool Tmc200::cmpt_real_data(tmc_data_t* tmc_data, tmc_real_data_t &tmc_real_data)
{
  bool res = true;
  if (!cmpt_tmcvel_to_radvel( tmc_data->V[0].data, tmc_data->V[1].data, tmc_data->V[2].data, 
			      tmc_real_data.V[0].data, tmc_real_data.V[1].data, tmc_real_data.V[2].data)) res = false;
  for (int i=0; i<3; i++)
    {
      tmc_real_data.V[i].tv.tv_sec = tmc_data->V[i].tv.tv_sec;
      tmc_real_data.V[i].tv.tv_usec= tmc_data->V[i].tv.tv_usec;
      
      // Stromumrechnung bei altem Board 0-10A bei neuem ohne Jumper 0-5A
      if (params.tmc_version < 1)
	tmc_real_data.C[i].data      = (tmc_data->C[i].data / 1024.0) * 10;
      else
	tmc_real_data.C[i].data      = (tmc_data->C[i].data / 1024.0) * 5;
      tmc_real_data.C[i].tv.tv_sec = tmc_data->V[i].tv.tv_sec;
      tmc_real_data.C[i].tv.tv_usec= tmc_data->V[i].tv.tv_usec;

      tmc_real_data.D[i].data      = tmc_data->D[i].data; 
      tmc_real_data.D[i].tv.tv_sec = tmc_data->V[i].tv.tv_sec;
      tmc_real_data.D[i].tv.tv_usec= tmc_data->V[i].tv.tv_usec;

      tmc_real_data.O[i].data      = (tmc_data->O[i].data / 1024.0) * 1; 
      tmc_real_data.O[i].tv.tv_sec = tmc_data->V[i].tv.tv_sec;
      tmc_real_data.O[i].tv.tv_usec= tmc_data->V[i].tv.tv_usec;

      tmc_real_data.T[i].data      = tmc_data->T[i].data; 
      tmc_real_data.T[i].tv.tv_sec = tmc_data->T[i].tv.tv_sec;
      tmc_real_data.T[i].tv.tv_usec= tmc_data->T[i].tv.tv_usec;
    }

  tmc_real_data.n.data        = tmc_data->n.data;
  tmc_real_data.n.tv.tv_sec   = tmc_data->n.tv.tv_sec;
  tmc_real_data.n.tv.tv_usec  = tmc_data->n.tv.tv_usec;

  tmc_real_data.dt.data       = tmc_data->dt.data * 10;
  tmc_real_data.dt.tv.tv_sec  = tmc_data->dt.tv.tv_sec;
  tmc_real_data.dt.tv.tv_usec = tmc_data->dt.tv.tv_usec;

  tmc_real_data.BE.data       = tmc_data->BE.data;
  tmc_real_data.BE.tv.tv_sec  = tmc_data->BE.tv.tv_sec;
  tmc_real_data.BE.tv.tv_usec = tmc_data->BE.tv.tv_usec;

  tmc_real_data.UB.data       = (tmc_data->UB.data > 0 ? (tmc_data->UB.data / 24.0) - 1.5 : 0);
  tmc_real_data.UB.tv.tv_sec  = tmc_data->UB.tv.tv_sec;
  tmc_real_data.UB.tv.tv_usec = tmc_data->UB.tv.tv_usec;

  tmc_real_data.SS[0].data       = (tmc_data->SS.data > 0 ? ((tmc_data->SS.data & 4) > 0 ? 1 : 0) : 0);
  tmc_real_data.SS[1].data       = (tmc_data->SS.data > 0 ? ((tmc_data->SS.data & 2) > 0 ? 1 : 0) : 0);
  tmc_real_data.SS[2].data       = (tmc_data->SS.data > 0 ? ((tmc_data->SS.data & 1) > 0 ? 1 : 0) : 0);
  for (int i=0; i<3; i++) {
    tmc_real_data.SS[i].tv.tv_sec  = tmc_data->SS.tv.tv_sec;
    tmc_real_data.SS[i].tv.tv_usec = tmc_data->SS.tv.tv_usec;
  }
  return res;
}


// blocking version of set velocity
bool Tmc200::setVelocity(int _v1_tmc, int _v2_tmc, int _v3_tmc, tmc_data_t &_data, timeval *tstmp)
{
  if (lost_connection)
    {
      reinit();
      return false;
    }

  sprintf(tmc_command,"SV %d %d %d\n",-_v1_tmc,-_v2_tmc,-_v3_tmc); // negativ -> Motoranschluss definition
  TMC_TALK(SerComI::T_SECOND);
  if (!tmc_com_res) {
    lost_connection = true;
    return false;
  }

  if (tstmp!=NULL)
    {
      tstmp->tv_sec=tmc_send_time.tv_sec;
      tstmp->tv_usec=tmc_send_time.tv_usec;
    }
  return parse_answer_string(tmc_answer, _data, &tmc_recv_time);
}


bool Tmc200::setVelocityRPS(float _v1, float _v2, float _v3, tmc_real_data_t &_data, timeval *tstmp)
{
  if (params.workingmode == 0)
    if (!setWorkingMode(1)) return false;
  bool res = true;
  int v[3];
  cmpt_radvel_to_tmcvel(_v1, _v2, _v3, v[0], v[1], v[2]);
  if (!setVelocity(v[0], v[1], v[2], tmc_data[0], tstmp)) res = false;
  if (!cmpt_real_data(&tmc_data[0], _data))       res = false;
  return res;
}

bool Tmc200::setVelocityU(float _v1, float _v2, float _v3, tmc_real_data_t &_data, timeval *tstmp)
{
  bool res = true;

  if (params.workingmode != 0)
    if (!setWorkingMode(0)) return false;
  
  int v[3];
  v[0] = (int) (_v1/params.vcc * 1024);
  v[1] = (int) (_v2/params.vcc * 1024);
  v[2] = (int) (_v3/params.vcc * 1024);

  if (!setVelocity(v[0], v[1], v[2], tmc_data[0], tstmp)) res = false;
  if (!cmpt_real_data(&tmc_data[0], _data))       res = false;
  return res;
}


// non blocking version of set velocity .. you have to call get data cyclic!!
bool Tmc200::setVelocity(int v1, int v2, int v3, timeval *tstmp)
{
  if(not_received_async_cmds.size() > 20)
    not_received_async_cmds.pop();
  if (  cmd_counter > 20)
    {
      lost_connection = true;
      THIS_ERROUT("MAXIMAL NUMBER OF ASYNC COMMANDS WITHOUT ANSWER REACHED!");
    }
  if (lost_connection)
    {
      reinit();
      return false;
    }

  sprintf(tmc_command,"SV %d %d %d\n",-v1,-v2,-v3); // negativ -> Motoranschluss definition
  TMC_SEND;
  if (!tmc_com_res) return false;
  
  TLabeledData cmd;
  cmd.data = ASYNC_CMD_VEL;
  cmd.tv.tv_sec = tmc_send_time.tv_sec;
  cmd.tv.tv_usec = tmc_send_time.tv_usec;
  not_received_async_cmds.push(cmd);
  cmd_counter ++;
  loopcounter ++;
 
  if (tstmp!=NULL)
    {
      tstmp->tv_sec=tmc_send_time.tv_sec;
      tstmp->tv_usec=tmc_send_time.tv_usec;
    }
  return true;
}


bool Tmc200::setVelocityRPS(float _v1, float _v2, float _v3, timeval *tstmp)
{
  if (params.workingmode == 0) {
    setWorkingMode_async(1);
    // we need one time step for command to execute on board
    return false;
  }

  int v[3];
  cmpt_radvel_to_tmcvel(_v1, _v2, _v3, v[0], v[1], v[2]);
  return setVelocity(v[0], v[1], v[2], tstmp);
}

bool Tmc200::setVelocityU(float _v1, float _v2, float _v3, timeval *tstmp)
{
  if (params.workingmode != 0) {
    setWorkingMode_async(0);
    // we need one time step for command to execute on board
    return false;
  }
  
  int v[3];
  v[0] = (int) (_v1/params.vcc * 1024);
  v[1] = (int) (_v2/params.vcc * 1024);
  v[2] = (int) (_v3/params.vcc * 1024);

  return setVelocity(v[0], v[1], v[2], tstmp);
}

// data retrival for non blocking version ... if not called cyclic buffer over run in serial line will crash system!!
int Tmc200::getTmcData(tmc_data_t * _data_list, int _max_num, bool _all )
{
  int n=0;
  bool res = true;
  int timeout_us;
  
  if (_all) timeout_us = 20000;
  else timeout_us=3000;

  while ( sercom->readline_async(tmc_answer, TMC_COM_BUF_SIZE - 1, timeout_us, &tmc_recv_time)
	  && (n < _max_num-1) )
    {
      if (!not_received_async_cmds.empty())
	{
	  if (not_received_async_cmds.front().data == ASYNC_CMD_VEL)
	    {
	      if (!parse_answer_string(tmc_answer, _data_list[n], &not_received_async_cmds.front().tv)) 
		res = false;
	      else
		{
		  if (cmd_counter >0 ) cmd_counter--;
		  not_received_async_cmds.pop();
		  n++;
		}
	    }
	  else 
	    if (not_received_async_cmds.front().data == ASYNC_CMD_PORT1L || 
		not_received_async_cmds.front().data == ASYNC_CMD_WM)
	      {
		if (tmc_answer[0]=='O' && tmc_answer[1]=='K')
		  {
		    not_received_async_cmds.pop();
		    if (cmd_counter >0 )
		      cmd_counter--;
		  }
		else
		  {
		    THIS_ERROUT("RECEIVED" << tmc_answer << " but not OK");
		    res = false;
		  }
	      }
	    else
	      {
		// some more Async cmds to implement
		// if some answers are missed here should happen a error recovery
		THIS_ERROUT("Some unknown data in async buffer!");
	      }
	}
      else
	{
	  THIS_ERROUT("No cmd in async buffer but receiving one! This should not happen!");
	}
    }
  if (n>=_max_num-1) THIS_ERROUT("Buffer overflow! More answers could be received than max buffer num given.");
  if (!res) THIS_ERROUT("Problems with parsing some answers.");

  if ((loopcounter % 20 == 0) && (cmd_counter>0))
    { 
      cmd_counter--;
    }
  return n;
}


int Tmc200::getTmcData(tmc_real_data_t * _data_list, int _max_num, bool _all)
{
  bool res = true;
  int n=0;
  int max_num=_max_num;

  if (_max_num> MAX_NUM_DATA) 
    {
      THIS_ERROUT("You want possibly to read more than MAX_NUM_DATA values ... so please increase define! " << MAX_NUM_DATA);
      max_num = MAX_NUM_DATA;
    }

  n=getTmcData(tmc_data, max_num, _all);

  for (int i=0; i<n; i++)
    if (!cmpt_real_data(&tmc_data[i], _data_list[i])) res = false;

  if (n>0 && res) params.vcc = _data_list[n-1].UB.data;

  if (!res) THIS_ERROUT("Problems on computing real data values.");
  return n;
}
