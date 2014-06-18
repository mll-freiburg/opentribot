#include "SGoToPosEvadeObstaclesOld.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../Fundamental/PiecewiseLinearFunction.h"
#include <iostream>

namespace Tribots {

  using namespace std;

  SGoToPosEvadeObstaclesOld::SGoToPosEvadeObstaclesOld() 
    : Skill("SGoToEvadeObstacles"),
      headingController(4., 0.0000 , 0.00, 5.5 , -5.5)
  {}

  SGoToPosEvadeObstaclesOld::~SGoToPosEvadeObstaclesOld() throw()
  {}

  
  void 
  SGoToPosEvadeObstaclesOld::init(Vec _targetPos, double _driveVel, 
				  Vec _targetHeading, 
				  bool _avoidBall) throw(TribotsException)
  {
    targetPos=_targetPos;
    driveVel=_driveVel;
    targetHeading=_targetHeading;
    avoidBall=_avoidBall;
  }

  DriveVector 
  SGoToPosEvadeObstaclesOld::getCmd(const Time& t) throw(TribotsException)
  {
    RobotLocation robotLocation = 
      MWM.get_robot_location(t);
    BallLocation ballLocation = 
      MWM.get_ball_location(t);

    Frame2d world2robot =  WhiteBoard::getTheWhiteBoard()->getAbs2RelFrame(t);
    Frame2d robot2world =  WhiteBoard::getTheWhiteBoard()->getRel2AbsFrame(t);
    
    DriveVector dv;
    dv.vrot = 
      headingController.getAction(0,
				  (robotLocation.heading -
				   (targetHeading.angle()-Angle::quarter)).get_rad_pi(),
				  33, LOUT); // \TODO: looptime instead of 33

    double max_speed=driveVel;
    PiecewiseLinearFunction speedlowering;
    speedlowering.addVertex(0.2,0.05);
    speedlowering.addVertex(0.3,0.5);
    speedlowering.addVertex(1.,1.);
    
    double distance;
    distance=(world2robot* targetPos).length();
    if (distance<1000) 
      max_speed=max_speed*speedlowering.getValue(distance/1000);
       
    dv.vtrans=(world2robot* targetPos).normalize()* max_speed;
    dv.kick = 0;
    
    // We now have our desired translational and rotational drive vectors,
    // but now we may have to modify it in order evade obstacles...
    dv.vtrans =
      WhiteBoard::getTheWhiteBoard()->calculateEvasiveMovement(dv.vtrans,
                                                               targetPos,
							       t);
    return dv;
  }
}

