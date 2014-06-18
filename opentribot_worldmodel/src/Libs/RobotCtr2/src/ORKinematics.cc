#include "ORKinematics.h"
#include <cmath>

#define DEG2RAD( __X__ ) (__X__ * M_PI / 180.0)

using namespace RobotCtr2;

ORKinMath_Params::ORKinMath_Params()
{
  L1 = L2       = 0.185;
  deltaDeg      = 30;
  wheelRadius   = 0.04;
  maxWheelSpeed = 90;
}


RobotVecVelocity::RobotVecVelocity(float _vx, float _vy, float _vphi)
{
  vx = _vx; vy = _vy; vphi = _vphi;
}


RobotWheelVelocity::RobotWheelVelocity(float v1, float v2, float v3)
{
  vw[0] = v1;
  vw[1] = v2;
  vw[2] = v3;
}


ORKinMath::ORKinMath(ORKinMath_Params _params)
{
  params   = _params;
  
  sindelta = sin(DEG2RAD(params.deltaDeg));
  cosdelta = cos(DEG2RAD(params.deltaDeg));
}

RobotWheelVelocity ORKinMath::inverse(const RobotVecVelocity& vel, bool truncate)
{
  float _v1, _v2, _v3;

  _v1 = 1.0/params.wheelRadius * (-sindelta * vel.vx + cosdelta * vel.vy + params.L1 * vel.vphi);
  _v2 = 1.0/params.wheelRadius * (-sindelta * vel.vx - cosdelta * vel.vy + params.L1 * vel.vphi);
  _v3 = 1.0/params.wheelRadius * (            vel.vx                     + params.L2 * vel.vphi);

  RobotWheelVelocity w(_v1, _v2, _v3);
  
  if (truncate) {
    float max=0;
    for (int i=0; i<3; i++) if (fabs(w.vw[i])>max) max = fabs(w.vw[i]);

    if (max > params.maxWheelSpeed) {
      float fak = params.maxWheelSpeed / max;
      for (int i=0; i<3; i++) w.vw[i]*=fak;
    }
  }
  
  return w;
}

RobotVecVelocity   ORKinMath::direct(const RobotWheelVelocity& vel)
{
  float h1, h2, h3;
  float _xm, _ym, _phim;

  h1 = -params.L2 / (2.0 * (params.L1 + sindelta * params.L2));
  h2 = -params.L2 / (2.0 * (params.L1 + sindelta * params.L2));
  h3 =  params.L1 / ( params.L1 + sindelta *  params.L2);
  _xm =  params.wheelRadius * ( h1 * vel.vw[0] + h2 *  vel.vw[1] + h3 * vel.vw[2]);

  h1 =  1.0 / (2*cosdelta);
  h2 = -1.0 / (2*cosdelta);
  h3 =  0;
  _ym = params.wheelRadius * ( h1 * vel.vw[0] + h2 *  vel.vw[1] + h3 * vel.vw[2]);

  h1 =  1.0 / (2.0 * (params.L1 + sindelta * params.L2));
  h2 =  1.0 / (2.0 * (params.L1 + sindelta * params.L2));
  h3 =  sindelta /   (params.L1 + sindelta * params.L2);
  _phim = params.wheelRadius * ( h1 * vel.vw[0] + h2 *  vel.vw[1] + h3 * vel.vw[2]);

  return RobotVecVelocity(_xm, _ym, _phim);
}


#if 0
#include <iostream>

int main() {
  
  ORKinMath k;

  RobotVecVelocity v =  k.direct(RobotWheelVelocity(90, -90, 0));

  std::cout << "vel: " << v.vy << "\n";

  RobotWheelVelocity vw = k.inverse(RobotVecVelocity(5, 0, 10), true);

  std::cout << "velwheel: " << vw.vw[0] << "  " << vw.vw[1] << "  " << vw.vw[2] << "\n";

}

#endif
