#include "OmniRobotKin.h"
#include "ValueReader.h"
#include "my_err_msg.h"
#include <cmath>

const float OmniRobotKin::L1_m_def    = 0.175;
const float OmniRobotKin::L2_m_def    = 0.175;
const float OmniRobotKin::R_m_def     = 0.04;
const float OmniRobotKin::delta_deg_def = 30;

OmniRobotKin::OmniRobotKin(const char* _conf_fname, const char * _conf_chapter,
			   std::ostream *_errout ,  std::ostream *_infoout)
{
  errout  = _errout;
  infoout = _infoout;

  init_params();

  read_params(_conf_fname, _conf_chapter);

  delta_rad = delta_deg * M_PI / 180.0;
  sindelta = sin(delta_rad);
  cosdelta = cos(delta_rad);
}

OmniRobotKin::OmniRobotKin(float _L1_m, float _L2_m, float _delta_deg, float _R_m,
			   std::ostream *_errout, 
			   std::ostream *_infoout)
{
  errout  = _errout;
  infoout = _infoout;

  L1_m = _L1_m;
  L2_m = _L2_m;
  R_m  = _R_m;
  delta_deg=_delta_deg;
  delta_rad = _delta_deg * M_PI / 180.0;
  sindelta = sin(delta_rad);
  cosdelta = cos(delta_rad);
}

void OmniRobotKin::init_params()
{
  L1_m = L1_m_def;
  L2_m = L2_m_def;
  R_m  = R_m_def;
  delta_deg = delta_deg_def;
}


bool OmniRobotKin::read_params(const char * _fname, const char * _chapter)
{
  ValueReader vr;
  if (! vr.append_from_file (_fname, _chapter))
    {
      THIS_ERROUT("Problem with reading " << _fname << " in Chapter " << _chapter);
    }
  if (vr.get ("L1_m", L1_m) < 0 ) 
    THIS_ERROUT("Can't read param: L1_m, using default: "<< L1_m);
  if (vr.get ("L2_m", L2_m) < 0 ) 
    THIS_ERROUT("Can't read param: L2_m, using default: "<< L2_m);
  if (vr.get ("R_m", R_m) < 0 ) 
    THIS_ERROUT("Can't read param: R_m, using default: "<< R_m);
  if (vr.get ("DELTA_deg", delta_deg) < 0 ) 
    THIS_ERROUT("Can't read param: DELTA_deg, using default: "<< delta_deg);
  return true;
}

void OmniRobotKin::cmptInvKin_RobotFrame(float _xm, float _ym, float _phim, float & _v1,float & _v2,float & _v3)
{
  _v1 = 1.0/R_m * (-sindelta * _xm + cosdelta * _ym + L1_m * _phim);
  _v2 = 1.0/R_m * (-sindelta * _xm - cosdelta * _ym + L1_m * _phim);
  _v3 = 1.0/R_m * (            _xm                  + L2_m * _phim);
 
}

OmniRobotKin::~OmniRobotKin()
{
  (*errout) << std::flush;
  (*infoout) << std::flush;
}

void OmniRobotKin::cmptDirectKin_RobotFrame(float _v1,float _v2,float _v3, float & _xm, float & _ym, float & _phim)
{
  float h1, h2, h3;

  h1 = -L2_m / (2 * (L1_m + sindelta * L2_m));
  h2 = -L2_m / (2 * (L1_m + sindelta * L2_m));
  h3 =  L1_m / (L1_m + sindelta * L2_m);
  _xm = R_m * ( h1 * _v1 + h2 *  _v2 + h3 * _v3);

  h1 =  1.0 / (2*cosdelta);
  h2 = -1.0 / (2*cosdelta);
  h3 =  0;
  _ym = R_m * ( h1 * _v1 + h2 *  _v2 + h3 * _v3);

  h1 =  1.0 / (2 * (L1_m + sindelta * L2_m));
  h2 =  1.0 / (2 * (L1_m + sindelta * L2_m));
  h3 =  sindelta /  (L1_m + sindelta * L2_m);
  _phim = R_m * ( h1 * _v1 + h2 *  _v2 + h3 * _v3);
}
