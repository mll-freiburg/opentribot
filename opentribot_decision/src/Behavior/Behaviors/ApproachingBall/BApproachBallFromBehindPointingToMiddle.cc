#include "BApproachBallFromBehindPointingToMiddle.h"
#include "../../../WorldModel/WorldModel.h"

using namespace std;
namespace Tribots 
{

BApproachBallFromBehindPointingToMiddle::
BApproachBallFromBehindPointingToMiddle(Vec targetPosRelToOppGoal) throw ()
  : Behavior("BApproachBallFromBehindPointingToMiddle"), 
    skill(new SApproachMovingBall()), lookAtBall(true)
{
  const FieldGeometry& field =
    MWM.get_field_geometry();
  
  targetPos = Vec(0, field.field_length/2.) + targetPosRelToOppGoal;    
  transVelFactor=1.0;
}

BApproachBallFromBehindPointingToMiddle::
~BApproachBallFromBehindPointingToMiddle() throw ()
{}

void BApproachBallFromBehindPointingToMiddle::updateTactics (const TacticsBoard& tb) 
    throw () 
{
  // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
        transVelFactor=0.8;
  } else	  
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
       transVelFactor=0.5;
  } else
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
       transVelFactor=1.3;
  } else { // =normal
       transVelFactor=1.0;
  }
  if (tb[string("3DFeldspielerVerhalten")] == "aus") {
    lookAtBall = false;
  } else {
    lookAtBall = true;
  }
}

DriveVector 
BApproachBallFromBehindPointingToMiddle::getCmd(const Time& t) 
  throw(TribotsException)
{
  const BallLocation& ballLocation =
    MWM.get_ball_location(t);
  const RobotLocation& robotLocation =
    MWM.get_robot_location(t);

  Vec targetDriveDir = (targetPos - ballLocation.pos.toVec()).normalize(); 
  Vec targetHeading = lookAtBall ? (ballLocation.pos.toVec()-robotLocation.pos).normalize() : targetDriveDir;
  
  double maxVel = 2.5*transVelFactor;
  
  if ((ballLocation.pos - robotLocation.pos).length() < 1300.) {
    if (MWM.get_game_state().refstate == freePlay) { // Ball ist frei -> schnell
      maxVel = 2.*transVelFactor;
    }
    else {
      maxVel = 1.2*transVelFactor;   // langsame Anfahrt, weil Anstoß
    }
  }
  skill->setParameters(targetDriveDir, targetHeading, maxVel);
  return skill->getCmd(t);
}

void 
BApproachBallFromBehindPointingToMiddle::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}
void 
BApproachBallFromBehindPointingToMiddle::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}

}
