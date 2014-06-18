#include "BApproachBallDirectly.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <cmath>
using namespace std;
namespace Tribots 
{

BApproachBallDirectly::BApproachBallDirectly(const string& name) throw ()
  : Behavior(name), 
    skill(new SApproachMovingBall())
{
  transVelFactor=1.0;
}

BApproachBallDirectly::~BApproachBallDirectly() throw ()
{}

void BApproachBallDirectly::updateTactics (const TacticsBoard& tb) 
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
       transVelFactor=1.5;
  } else { // =normal
       transVelFactor=1.0;
  }
}

DriveVector 
BApproachBallDirectly::getCmd(const Time& t) 
  throw(TribotsException)
{
  
  double desiredVel;
  double ballDistance = MWM.get_ball_relative().pos.length();
  if (ballDistance > 1800.) {
    desiredVel = 2.5 * transVelFactor;
  }
  else {
    desiredVel = 1.5*transVelFactor;
  }

  skill->setParameters(MWM.get_ball_location(t).pos.toVec() - MWM.get_robot_location(t).pos, desiredVel);
  return skill->getCmd(t);
}

 bool BApproachBallDirectly::checkInvocationCondition (const Time& t) throw() {
   return true;
   
     Vec relBall=MWM.get_ball_relative().pos;
   
     if (fabs(relBall.x)<300  &&   fabs(relBall.y) < 600) return true;
 // LOUT<<"relball: "<<relBall<<"\n";

  return false;
}
bool BApproachBallDirectly::checkCommitmentCondition (const Time& t) throw() {
   return checkInvocationCondition(t);
}
void 
BApproachBallDirectly::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}
void 
BApproachBallDirectly::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}

}
