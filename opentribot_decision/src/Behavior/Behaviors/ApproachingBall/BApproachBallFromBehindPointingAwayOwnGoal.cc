#include "BApproachBallFromBehindPointingAwayOwnGoal.h"
#include "../../../WorldModel/WorldModel.h"

using namespace std;
namespace Tribots
{

BApproachBallFromBehindPointingAwayOwnGoal::BApproachBallFromBehindPointingAwayOwnGoal()
  throw()
  : Behavior("BApproachBallFromBehindPointingAwayOwnGoal"), 
    skill(new SApproachMovingBall()), lookAtBall(true)
{
  transVelFactor=1.0;
}

BApproachBallFromBehindPointingAwayOwnGoal::~BApproachBallFromBehindPointingAwayOwnGoal()
  throw()
{
  delete skill;
}

void BApproachBallFromBehindPointingAwayOwnGoal::updateTactics (const TacticsBoard& tb) 
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
        lookAtBall=false;
      } else {
        lookAtBall = true;
      }
}


DriveVector 
BApproachBallFromBehindPointingAwayOwnGoal::getCmd(const Time& t) 
  throw(TribotsException)
{
  const BallLocation& ballLocation =
    MWM.get_ball_location(t);
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& field =
    MWM.get_field_geometry();

  Vec ownGoalPos(0., -500-field.field_length / 2.);
  Vec targetDir = -ownGoalPos +ballLocation.pos.toVec(); // Zielausrichtung
  Vec targetHeading = lookAtBall ? (ballLocation.pos.toVec()-robot.pos).normalize() : targetDir;
 
  skill->setParameters(targetDir, targetHeading, 2.5*transVelFactor);
  return skill->getCmd(t);
}

void 
BApproachBallFromBehindPointingAwayOwnGoal::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}
void 
BApproachBallFromBehindPointingAwayOwnGoal::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}

};
