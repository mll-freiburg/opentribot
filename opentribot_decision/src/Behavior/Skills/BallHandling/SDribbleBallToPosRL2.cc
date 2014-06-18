#include "SDribbleBallToPosRL2.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <cmath>

using namespace Tribots;
using namespace std;


SDribbleBallToPosRL2::SDribbleBallToPosRL2(): SDribbleBallToPosInterface("SDribbleBallToPosRL2") {
  name = "SDribbleBallToPosRL2";
  controller		= 0;
  actual_world_target	= Vec(0,0);
  odim			= 5;
  adim			= 3; 

  controller = new RLDribbleControllerFixed2(odim,adim);

}

SDribbleBallToPosRL2::~SDribbleBallToPosRL2() throw()
{
  if (controller!=0) delete controller;
}

void SDribbleBallToPosRL2::get_observation(double *observation) throw(TribotsException){

  Time t; // besser übergeben !!

  const RobotLocation& robot = MWM.get_robot_location(t);
  Frame2d world2robot =  WBOARD->getAbs2RelFrame(t);
  
  Vec relRobotVel  = robot.vtrans.rotate(-robot.heading);
  Vec relTarget    = world2robot * actual_world_target;

  observation[0] = relRobotVel.x; // rel vel x m/s
  observation[1] = relRobotVel.y; // rel vel y m/s
  observation[2] = robot.vrot;    // vel phi rad/s
  // diff from heading to rel target -pi,pi:
  observation[3] = (relTarget.angle() - Angle::quarter).get_rad_pi (); 
  //  observation[4] = relTarget.length() * 0.001; // distance to target in m
  //  observation[4] = 0.0; // turn to zero differnce angle only

  if (WBOARD->doPossessBall(t)) {
    observation[odim -1] = 0;
  } else {
    observation[odim -1] = 1;
  }
}

DriveVector SDribbleBallToPosRL2::getCmd(const Time& t) throw(TribotsException){
  DriveVector dv;

  LOUT <<"% blue thin solid circle "
       <<actual_world_target.x<<" "<<actual_world_target.y <<" 50 word "
       <<actual_world_target.x+100<<" "<<actual_world_target.y+100<<" RL2:target\n";
  
  double observation[odim];
  double action[adim];

  get_observation(observation);
  controller->getCmd(observation, action);

  dv.vrot     = action[0]; // rad/s
  dv.vtrans.x = action[1]; // m/s
  dv.vtrans.y = action[2]; // m/s

  return dv;
}

void 
SDribbleBallToPosRL2::setParameters(const Vec& target, 
                                   double transVel, bool)
  throw(TribotsException)
{
  actual_world_target = target;
  controller->setGoStraightVelocity(transVel);
}





// ------------------------ hier RL -----------------------------------------------


RLDribbleControllerFixed2::RLDribbleControllerFixed2(unsigned int _odim, unsigned int _adim)
{
  odim = _odim;
  adim = _adim;

  // parameters:
  spec.gostraight_speed = 2.3;
  spec.gostraight_threshhold = 5.0/180. * 3.14159;

#define PATH_TO_NFQ_CONFIG "../config_files/rl_dribble/dribble2.nfq"
  spec.training = false;
  spec.gostraight_threshhold = 20.0/180. * 3.14159;
  nfqcontroller.init(odim, 1, PATH_TO_NFQ_CONFIG,  "DRIBBLE"); 
}

RLDribbleControllerFixed2::~RLDribbleControllerFixed2()
{
  ;
}

bool RLDribbleControllerFixed2::getCmd(const double* observation, double* action)
{
  bool result = false;
  double tmp_action[1];
  tmp_action[0] = 0.0; // default

  action[2] = 1.5;  // default let robot go by 1.5

#if 0  // turn control
  // simple turn control (for test positioning)
  result = nfqcontroller.get_action(observation,tmp_action);
  if(observation[3]<0)
    tmp_action[0] = -tmp_action[0];
  action[0] = tmp_action[0];

  //  action[0] = .4 * action[0];  // a bit aggresiv -> overshoot
  action[1] = .2 * action[0];  // determine x velocity by a simple P controller

#endif // test turn control
#if 1  // vx control

  // first, compute rotation speed control
  // simple P controller:
  //  action[0] = 2.5 * observation[3];  // determine angular velocity by a simple P controller
  // bang bang controller: (might be improved by considering distance also):
  float rotspeed = 0.;
  if(fabs(observation[3]) > 5./180.0 * 3.1415)
    rotspeed = 1.0;
  if(fabs(observation[3]) > 20./180.0 * 3.1415)
    rotspeed = 1.7; // normal
    //    rotspeed = 4.0;  // aggressiv
  
  if(observation[3]<0)
    rotspeed *= -1;
  action[0]  = rotspeed;

  // determine VX control
  result = nfqcontroller.get_action(observation,tmp_action);
  double vx = 0;
  double vy = 1.5;

  double aval = fabs(tmp_action[0]);
  int asign;
  if(tmp_action[0] >=0)
    asign = 1;
  else
    asign = -1;
  if(aval == 0.0){
    vx = 0;
    vy = 0;
  }
  else if(aval == 1.0){
    vx = 0;
    vy = 1.5;
  }
  else if(aval == 2.0){
    vx = 0;
    vy = 2.0;
  }
  else if(aval == 3.0){
    vx = 0;
    vy = 2.5;
  }
  else if(aval == 4.0){
    vx = 1.06;
    vy = 1.06;
  }
  else if(aval == 5.0){
    vx = 1.4;
    vy = 1.4;
  }
  else if(aval == 6.0){
    vx = 1.8;
    vy = 1.8;
    // avoid overshooting
    vx = 1.5;
    vy = 1.5;
  }
  else if(aval == 7.0){
    vx = 1.5;
    vy = 0;
  }
  else if(aval == 8.0){
    vx = 2.0;
    vy = 0;
  }
  else if(aval == 9.0){
    vx = 2.5;
    vy = 0;
  }
  vx = asign * vx;

  if (action[0]<0)  // desired rotspeed is negative 
  //  if (observation[2]<0)  // current rotspeed is negative 
    vx =  -vx;

  action[2] = vy;
  action[1] = vx;

#if 1 // aggressiv: accelerate towards goal, if direction is ok
  if(fabs(observation[3]) < spec.gostraight_threshhold){
    action[2] = spec.gostraight_speed;
    action[1] = 0;
  }
#endif

#endif

#define ROTVEL_MAX 5.0
  // restrict max rotation speed
  if(action[0] > ROTVEL_MAX)
    action[0] = ROTVEL_MAX;
  if(action[0] < -ROTVEL_MAX)
    action[0] = -ROTVEL_MAX;

#define VX_MAX 5.0
  if(action[1] > VX_MAX )
    action[1] = VX_MAX ;
  if(action[1] < -VX_MAX )
    action[1] = -VX_MAX ;

  LOUT << "Fixed RL controller 2: NFQ Action: "<<tmp_action[0]<<" rot.vel: "<<action[0]
       <<" xvel: "<<action[1] <<" yvel: "<<action[2] << "\n";
  return result;
}
