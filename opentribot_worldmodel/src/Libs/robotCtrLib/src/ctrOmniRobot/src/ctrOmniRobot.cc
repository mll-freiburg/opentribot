#include "ctrOmniRobot.h"
#include "my_err_msg.h"
#include "ValueReader.h"
#include <cmath>

const float ctrOmniRobot::max_wheel_vel_def   = 35.0; //rad/s
const float ctrOmniRobot::max_wheel_acc_def   = 50.0; //rad/s*s
const float ctrOmniRobot::max_wheel_deacc_def = 60.0; //rad/s*s

std::ostream& operator << (std::ostream& os, ctrOmniRobotData_t& data)
{
  os << "v(" << data.tmc_version << ") " 
     << "wv:(" <<  data.wheel_vel[0] << "," <<  data.wheel_vel[1] << "," <<  data.wheel_vel[2] << ") "
     << "rv:(" <<  data.robot_vel[0] << "," <<  data.robot_vel[1] << "," <<  data.robot_vel[2] << ") "
     << "c:(" <<  data.current[0] << "," <<  data.current[1] << ","<<  data.current[2] << ") "
     << "ts:(" <<  data.temp_switch[0] << "," <<  data.temp_switch[1] << "," <<  data.temp_switch[2] << ") "
     << "t:(" <<  data.temp[0] << "," <<  data.temp[1] << "," <<  data.temp[2] << ") "
     << "vcc:(" << data.vcc << ")";
  return os;
}


ctrOmniRobot::ctrOmniRobot(const char* _conf_fname, const char * _conf_chapter, 
			   std::ostream *_errout ,  std::ostream *_infoout)
{
  errout  = _errout;
  infoout = _infoout;
  
  board = 0;
  kin   = 0;

  board = new Tmc200(_conf_fname, errout, infoout);
  board->print_params(*infoout);

  kin = new OmniRobotKin(_conf_fname, "OMNIKIN", errout, infoout);

  read_params(_conf_fname, _conf_chapter);

  // check params
  for (int i=0; i<3; i++)
    {
      float max_motor_vel = board->get_max_motor_vel(i);
      if (max_wheel_vel > max_motor_vel)
	{
	  THIS_ERROUT("Parameter max_wheel_vel "<< max_wheel_vel << "bigger than speed that can be reached by motor("<< i << "): " << max_motor_vel);
	  max_wheel_vel = max_motor_vel;
	}
    }
  THIS_INFOOUT("Params ok, we can init now ...");
  inited = false;
}

bool ctrOmniRobot::init() {
  inited = board->init();
  return inited;
}

void ctrOmniRobot::read_params(const char * _fname, const char * _chapter)
{
  ValueReader vr;
  if (! vr.append_from_file (_fname, _chapter))
    {
      THIS_ERROUT("Problem with reading " << _fname << " in Chapter " << _chapter);
    }
  if (vr.get ("max_wheel_vel", max_wheel_vel) < 0 ) 
    {
      THIS_ERROUT("Can't read param: max_wheel_vel, using default: "<< max_wheel_vel_def);
      max_wheel_vel = max_wheel_vel_def;
    }
  if (vr.get ("max_wheel_acc", max_wheel_acc) < 0 ) 
    {
      THIS_ERROUT("Can't read param: max_wheel_acc, using default: "<< max_wheel_acc_def);
      max_wheel_acc = max_wheel_acc_def;
    }
  if (vr.get ("max_wheel_deacc", max_wheel_deacc) < 0 ) 
    {
      THIS_ERROUT("Can't read param: max_wheel_deacc, using default: "<< max_wheel_deacc_def);
      max_wheel_deacc = max_wheel_deacc_def;
    }
}

ctrOmniRobot::~ctrOmniRobot()
{
  if (board) delete board;
  if (kin)   delete kin;

  (*errout) << std::flush;
  (*infoout) << std::flush;
}


bool ctrOmniRobot::setVel_vec_nonblocking(float _xm, float _ym, float _phim, timeval *tstmp,
					  float *v1, float* v2, float* v3)
{
  float vw1, vw2, vw3;
  
  kin->cmptInvKin_RobotFrame(_xm, _ym, _phim, vw1, vw2, vw3);
  if (v1!=0) *v1=vw1;
  if (v2!=0) *v2=vw2;
  if (v3!=0) *v3=vw3;
  
  bool res = setVel_wheels_nonblocking(vw1, vw2, vw3, tstmp);
  
  return res;
}

bool ctrOmniRobot::setVel_vec_blocking(float _xm, float _ym, float _phim, 
				       ctrOmniRobotData_t * data, timeval *tstmp,
				       float *v1, float* v2, float* v3)
{
  float vw1, vw2, vw3;
  kin->cmptInvKin_RobotFrame(_xm, _ym, _phim, vw1, vw2, vw3);
  if (v1!=0) *v1=vw1;
  if (v2!=0) *v2=vw2;
  if (v3!=0) *v3=vw3;
  bool res = setVel_wheels_blocking(vw1, vw2, vw3, data,  tstmp);

  return res;
}

bool ctrOmniRobot::setVel_wheels_nonblocking(float _v1, float _v2, float _v3, timeval *tstmp)
{
  if (!inited) {
    THIS_ERROUT("Board not inited ... !");
    return false;
  }

  bool res;  
  res = board->setVelocityRPS(_v1, _v2, _v3, tstmp);
  if (!res) 
    THIS_ERROUT("Problems with setting wheel vel!");
  return res;
}


bool ctrOmniRobot::setVel_wheels_blocking(float _v1, float _v2, float _v3, 
					  ctrOmniRobotData_t * data, timeval *tstmp)
{
  if (!inited) {
    THIS_ERROUT("Board not inited ... !");
    return false;
  }
  
  tmc_real_data_t tmc_data;
  bool res;  
  res = board->setVelocityRPS(_v1, _v2, _v3, tmc_data, tstmp);
  cmptOmnirobotData(*data, tmc_data);
  if (!res) 
    THIS_ERROUT("Problems with setting wheel vel!");
  return res;
}

bool ctrOmniRobot::setVoltage_motors_blocking(float _v1, float _v2, float _v3, 
					      ctrOmniRobotData_t * data, timeval *tstmp)
{
  if (!inited) {
    THIS_ERROUT("Board not inited ... !");
    return false;
  }
  tmc_real_data_t tmc_data;
  bool res;  
  res = board->setVelocityU(_v1, _v2, _v3, tmc_data, tstmp);
  cmptOmnirobotData(*data, tmc_data);
  if (!res) 
    THIS_ERROUT("Problems with setting motor voltage!");
  return res;
}
bool ctrOmniRobot::setVoltage_motors_nonblocking(float _v1, float _v2, float _v3, timeval *tstmp)
{
  if (!inited) {
    THIS_ERROUT("Board not inited ... !");
    return false;
  }

  bool res;  
  res = board->setVelocityU(_v1, _v2, _v3, tstmp);
  if (!res) 
    THIS_ERROUT("Problems with settingmotor voltage!");
  return res;
}

int ctrOmniRobot::getData_nonblocking(ctrOmniRobotData_t * datalist, int maxnum)
{
  if (!inited) {
    THIS_ERROUT("Board not inited ... !");
    return 0;
  }

  tmc_real_data_t tmc_data[10];
  int n = board->getTmcData(tmc_data, 10);

  if (n>maxnum) n=maxnum;

  int h=0;
  for (int i=n-1; i>=0; i--)
    {
      cmptOmnirobotData(datalist[h], tmc_data[i]);
      h++;
    }
  return n;
}


void ctrOmniRobot::cmptOmnirobotData(ctrOmniRobotData_t & target, tmc_real_data_t tmc_src)
{
  for (int i=0; i<3; i++) target.wheel_vel[i] = tmc_src.V[i].data;
  target.timestamp_vel = tmc_src.V[0].tv;
  kin->cmptDirectKin_RobotFrame(tmc_src.V[0].data, tmc_src.V[1].data, tmc_src.V[2].data,
				target.robot_vel[0], target.robot_vel[1], target.robot_vel[2]);

  for (int i=0; i<3; i++) target.current[i] = tmc_src.C[i].data;
  target.timestamp_current = tmc_src.C[0].tv;

  target.tmc_version = board->get_tmc_version();

  for (int i=0; i<3; i++) target.output[i] = tmc_src.O[i].data;

  for (int i=0; i<3; i++) target.temp_switch[i]=tmc_src.SS[i].data;
  target.timestamp_temp_switch = tmc_src.SS[0].tv;

  for (int i=0; i<3; i++) target.temp[i]=tmc_src.T[i].data;
  target.timestamp_temp = tmc_src.T[0].tv;
  
  target.vcc = tmc_src.UB.data;
  target.timestamp_vcc = tmc_src.UB.tv;
}

bool ctrOmniRobot::setKickPort_blocking(bool high, timeval *tstmp)
{
  if (!inited) {
    THIS_ERROUT("Board not inited ... !");
    return 0;
  }
  
  int v;
  if (high) v=3; else v=0; // mit Befehl port1l werden 3 ports geschaltet ... welcher?

  bool res= board -> set_port1L( v , tstmp);
  if (!res) 
    THIS_ERROUT("Problems with setting kicker port!");
  return res;
}

bool ctrOmniRobot::setKickPort_nonblocking(bool high, timeval *tstmp)
{
  if (!inited) {
    THIS_ERROUT("Board not inited ... !");
    return 0;
  }
  
  int v;
  if (high) v=3; else v=0; // mit Befehl port1l werden 3 ports geschaltet ... welcher?
  
  bool res= board -> set_port1L_async( v , tstmp);
  if (!res) 
    THIS_ERROUT("Problems with setting kicker port (async)!");
  return res;

}

bool ctrOmniRobot::get_realizable_vel_vec(float _act_xm, float _act_ym, float _act_phim,  
					  float _new_xm, float _new_ym, float _new_phim, float _dt_ms,
					  float & _realizable_xm, float & _realizable_ym, float & _realizable_phim)
{
  /* A) ein Rad (oder mehrere) überschreiten max Rad-Geschwindigkeit
   * -> versuche Bahn einzuhalten und skaliere die Radgeschwindigkeiten entsprechend
   */

  float vw[3];
  kin->cmptInvKin_RobotFrame(_new_xm, _new_ym, _new_phim, vw[0], vw[1], vw[2]);

  float max_vel=0;
  for (int i=0; i<3; i++) 
    if ( fabs(vw[i]) > fabs(max_vel) ) max_vel=vw[i];


  float scale_vel=1.0;
  if (fabs(max_vel) > max_wheel_vel) 
    scale_vel = max_wheel_vel / fabs(max_vel);
  
  for (int i=0; i<3; i++)
    vw[i]=scale_vel * vw[i];

  /* B) ein Rad überschreitet die max Beschleunigung
   * -> welchen Vektor soll gefahren werden?
   */
  float vw_old[3];
  kin->cmptInvKin_RobotFrame(_act_xm, _act_ym, _act_phim, vw_old[0], vw_old[1], vw_old[2]);

  float vw_diff[3];
  float vw_max_diff=0;
  for (int i=0; i<3; i++)
    {
      vw_diff[i]=vw[i]-vw_old[i];
      if (fabs(vw_diff[i]) > fabs(vw_max_diff)) vw_max_diff = vw_diff[i];
    }
  
  float acc_scale = 1.0;

  bool acc[3];
  for (int i=0; i<3; i++) acc[i] = (vw_diff[i] * vw_old[i]) > 0;

  if (acc[0] && acc[1] && acc[2]) // all wheels accelerated by acc
    {
      if (fabs(vw_max_diff)>(max_wheel_acc * (_dt_ms/1000.0)))
	acc_scale = (max_wheel_acc* (_dt_ms/1000.0)) / fabs(vw_max_diff);
    }
  else if (!acc[0] && !acc[1] && !acc[2]) // all wheels deaccelerated by deacc
    {
      if (fabs(vw_max_diff)>(max_wheel_deacc * (_dt_ms/1000.0)))
	acc_scale = (max_wheel_deacc* (_dt_ms/1000.0)) / fabs(vw_max_diff);
    }
  else // acceleration and deacceleration mixed by smallest value
    {
      float acc;
      if (max_wheel_deacc < max_wheel_acc) acc=max_wheel_acc; else acc=max_wheel_deacc;
      if (fabs(vw_max_diff)>(acc * (_dt_ms/1000.0)))
	acc_scale = (acc* (_dt_ms/1000.0)) / fabs(vw_max_diff);
    }
  
  float _realizable_vw[3];
  for (int i=0; i<3; i++) 
    {
      _realizable_vw[i]=vw_old[i]+(acc_scale * vw_diff[i]);
    }

  kin->cmptDirectKin_RobotFrame(_realizable_vw[0],_realizable_vw[1] ,_realizable_vw[2],
				_realizable_xm, _realizable_ym, _realizable_phim);
  
  return true;
}
