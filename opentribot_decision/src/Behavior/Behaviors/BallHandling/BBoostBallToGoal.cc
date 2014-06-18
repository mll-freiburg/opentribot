#include "BBoostBallToGoal.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/stringconvert.h"
#include "../../../Player/WhiteBoard.h"

#include <stdlib.h>
#define drand48 (1.0/((double) RAND_MAX)) *rand



using namespace Tribots;
using namespace std;

BBoostBallToGoal::BBoostBallToGoal()
  : Behavior("BBoostBallToGoal"), 
    skill(new SBoostToPos(1500.0 , true))
{
  skill->setTargetPos(Vec (0., MWM.get_field_geometry().field_length / 2.)); // opponent goal
  exec_prob			= 0.5;
  last_cycle_ball_possession	= false;
  licence			= false;
}

BBoostBallToGoal::
~BBoostBallToGoal() throw ()
{
  delete skill;
}

void BBoostBallToGoal::updateTactics (const TacticsBoard& tb) throw () 
{
  double d=0.5;
  if (string2double (d, tb[string("Boost2Goal")])) {
    if (d>=0 && d<=1) exec_prob=d;
  }

  LOUT << "Tactics changed in BBoostBallToGoal probability=" << exec_prob << std::endl;
}

bool BBoostBallToGoal::checkCommitmentCondition(const Time& t) throw()
{
  bool res = true;

  if (!licence) return false; 

  const FieldGeometry& field  = MWM.get_field_geometry();
  const RobotLocation& robot  = MWM.get_robot_location(t);
    
  // bis zur Mitte der gegnerischen haelfte
  res &= robot.pos.y < (field.field_length/4.);

  // Roboter hat Ball , diff Winkel zu Target nicht zu gross , keine (unbekannten) Hindernisse im Weg
  res &= skill->checkInvocationCondition(t);

  // TODO: evtl. weitere Kriterien (zu nah am Tor bzw. goal area, ... )

  return res;    
}

bool BBoostBallToGoal::checkInvocationCondition(const Time& t) throw()
{
  return 
    MWM.get_robot_location(t).pos.y < MWM.get_field_geometry().field_length / 9. &&
    BBoostBallToGoal::checkCommitmentCondition(t);
}

DriveVector BBoostBallToGoal::getCmd(const Time& t) throw(TribotsException) 
{
  return skill->getCmd(t);
}
  
  
void BBoostBallToGoal::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}

void BBoostBallToGoal::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}

void BBoostBallToGoal::cycleCallBack(const Time& t) throw()
{
  bool ballPossesion = WBOARD->doPossessBall(t);
  if (!last_cycle_ball_possession && ballPossesion) { // gerade den Ball erhalten?
    double r = drand48();
    licence =  (r < exec_prob);                 // Verhalten diesmal aktiv?
    LOUT << "<" <<  __PRETTY_FUNCTION__ << "> : " << "Würfeln: " << licence << " (prob: " << r << " , " << exec_prob << ")\n";
  }
  last_cycle_ball_possession = ballPossesion;
}
