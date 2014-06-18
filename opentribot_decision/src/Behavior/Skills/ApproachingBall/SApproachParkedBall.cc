#include "SApproachParkedBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

namespace Tribots
{ 
  using namespace std;
  
  SApproachParkedBall::SApproachParkedBall(double maxVel) throw ()
    : Skill ("SApproachParkedBall"), 
      headingController(4., 0.0000 , 0.00, 5.5 , -5.5),
      approachTarget(),
      approachSpeeds(),
      maxVel(maxVel),
      desiredVel(maxVel)
  {
    approachFacing_exp_smoothed=Vec(0,0);
    headingController.reset();
    
    approachTarget.addVertex(0.2, 0.5);
    approachTarget.addVertex(0.5, 0.5);
    
    approachSpeeds.addVertex(0.3, 0.4);
    approachSpeeds.addVertex(0.6, 0.6);

    lowVelMod.addVertex(.1, .6);      // schneller Anstieg
    lowVelMod.addVertex(1., 1.);      // sehr langsamer Anstieg bis 1
  }
  
  void 
  SApproachParkedBall::loseControl(const Time&) throw(TribotsException)
  {
    headingController.reset();
  }
  
  void SApproachParkedBall::setParameters(const Vec& targetdir,
					  double desiredVel) 
    throw(TribotsException)
  {
    approachFacing=targetdir;
    this->desiredVel = desiredVel;
    if (desiredVel > maxVel) {
      this->desiredVel = maxVel;
    }
  }

  DriveVector SApproachParkedBall::getCmd(const Time& tt) 
    throw(TribotsException)
  {
    Time t;
    t=tt;
    t.add_msec(-100);        // \todo: Dirty hack, need to repair this

    Angle targetHeading=approachFacing.angle();
    Vec targetMovement=approachFacing;

    RobotLocation robotLocation = 
      MWM.get_robot_location(t);

    Angle robotHeading = robotLocation.heading;
    Vec ballPos = MWM.get_ball_location(t).pos.toVec();
    
    //SVEN TODO URGENT
    
    if(MWM.get_game_state().refstate == preOwnKickOff){
      ballPos.x = 500;
      ballPos.y = -288;
    }
    if(ballPos.x > 700){
      ballPos.x = ballPos.x+200;
    }else{
      ballPos.x = ballPos.x-200;
    }
    Vec robotPos=robotLocation.pos;
    Frame2d world2robot =  WhiteBoard::getTheWhiteBoard()->getAbs2RelFrame(t);
    Frame2d robot2world =  WhiteBoard::getTheWhiteBoard()->getRel2AbsFrame(t);

    DriveVector dv;
    dv.kick = 0;
    dv.vrot =                   // calculate correction of heading
      headingController.getAction(0,
                                  (robotHeading -
                                   (targetHeading-Angle::quarter)).get_rad_pi(),
                                  MWM.get_game_state().intended_cycle_time, 
                                  LOUT);
   
    dv.vrot = lowVelMod.getValue(fabs(dv.vrot)) * (fabs(dv.vrot) / dv.vrot);
    
    // Calculate an approaching vector

    Angle theta=(ballPos-robotPos).angle(targetMovement);
     

    Vec evade = 1000.*approachTarget.getValue(fabs(theta.get_rad_pi())/M_PI)*
        targetMovement.normalize().rotate(M_PI-theta.get_rad_pi()*0.5);
         // \todo: Was macht das genau?
    Vec dogCurveTarget = ballPos + evade;
    
    LOUT << "\% red solid cross " << dogCurveTarget 
         << " blue cross " << " word "<<Vec(0,500)+dogCurveTarget 
         <<" DogCurveTarget,Theta" <<theta.get_rad_pi()<<'\n';

    double dist=(ballPos-robotPos).length();
    double brake=approachSpeeds.getValue(dist/1000.);
    if (dist > 2000.) {
      brake = 1.;       // do not brake
    }else{
      brake = (dist-200)/2000;
    }
   
    Vec desiredVtrans = 
      (world2robot*dogCurveTarget).normalize()*desiredVel*brake;

    if (desiredVtrans.length() > maxVel) { // cut vtrans at max speed
      desiredVtrans = desiredVtrans.normalize()*maxVel;
    }
    
    // We now have our desired translational and rotational drive vectors, 
    // but now we may have to modify it in order evade obstacles...

    dv.vtrans = 
      WhiteBoard::getTheWhiteBoard()->calculateEvasiveMovement(desiredVtrans,
                                                               dogCurveTarget, 
                                                               tt);
                                                               
    return dv;
  }
    
}

