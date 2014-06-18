#ifndef _ORKINEMATICS_H_
#define _ORKINEMATICS_H_

namespace RobotCtr2 {

struct ORKinMath_Params 
{
  ORKinMath_Params();
  float L1;          //[m]
  float L2;          //[m]
  float deltaDeg;    //[degrees]
  float wheelRadius; //[m]
  float maxWheelSpeed; //[rad/s]
};

struct RobotVecVelocity {
  RobotVecVelocity(float _vx=0, float _vy=0, float _vphi=0);
  float vx;          // [m/s]
  float vy;          // [m/s]
  float vphi;        // [rad/s]
};

struct RobotWheelVelocity {
  RobotWheelVelocity(float v1=0, float v2=0, float v3=0);
  float vw[3];       // [rad/s]
};

class ORKinMath
{
 public:
  ORKinMath(ORKinMath_Params _params=ORKinMath_Params());

  RobotWheelVelocity inverse(const RobotVecVelocity& vel, bool truncate=false);
  RobotVecVelocity   direct(const RobotWheelVelocity& vel);

 protected:
  ORKinMath_Params params;

  float sindelta;
  float cosdelta;

};

}

#endif
