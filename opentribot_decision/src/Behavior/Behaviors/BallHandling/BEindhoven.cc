#include "BEindhoven.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include <cmath>
#include <iostream>

namespace Tribots
{
using namespace std;

BEindhoven::
BEindhoven(double transVel)
  : Behavior("BEindhoven"), 
    activated(false), cycles_on_target(0), transVel(transVel),
    skill(new SDribbleBallToPos())
{}

BEindhoven::
~BEindhoven() throw ()
{
  delete skill;
}

void BEindhoven::updateTactics (const TacticsBoard& tb) 
    throw () 
{
    delete skill;
    skill = new SDribbleBallToPos();
  
  if (tb[string("EindhovenMove")] == "an") {
    activated = true;
  }
  else {
    activated = false;
  }

  // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
        transVel=1.4;
  } else	  
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
       transVel=1.0;
  } else
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
       transVel= 2.3;
  } else { // =normal
       transVel= 1.8;
  }

  LOUT << "In BEindhoven tactics changed. New skill: " 
       << skill->getName() << ". transVel=" << transVel << "\n";
}

bool 
BEindhoven::
checkCommitmentCondition(const Time& t) throw()
{
  if (MWM.get_robot_properties().kickers < 2) {
    return false;
  }
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& field = MWM.get_field_geometry();
  if (! WBOARD->doPossessBall(t)) {
    return false;
  }
  if (! activated) { 
    return false;
  }
  if (robot.pos.y < -field.field_length/2.+field.penalty_area_length+2000 ||
      robot.pos.y > +field.field_length/2.-1500.) {
    return false;
  }
  return true;    
}

bool 
BEindhoven::
checkInvocationCondition(const Time& t) throw()
{
  return BEindhoven::checkCommitmentCondition(t);
}

DriveVector 
BEindhoven::getCmd(const Time& t) 
  throw(TribotsException) 
{
  FieldGeometry const& field= MWM.get_field_geometry();
  
  // \TODO : gets the "unfiltered" robot location 
  Time now;
  RobotLocation robot = MWM.get_slfilter_robot_location(now); // achtung: diese methode schreibt in now!
  
  Vec targetPos(0., field.field_length / 2.);   
  skill->setParameters(targetPos, transVel, false);
  DriveVector dv = skill->getCmd(t);
  vector<Vec> goal_line_points = 
      intersect(LineSegment(Vec(-field.goal_width/2 + 200., field.field_length/2.),
                            Vec(+field.goal_width/2 - 200., field.field_length/2.)),
                LineSegment(robot.pos, robot.pos + (Vec(0,2.) * robot.heading) * field.field_length));
  if (goal_line_points.size() > 0) {
    cycles_on_target++;
  }
  else {
    cycles_on_target = 0;
  }
  
  Quadrangle inFront(robot.pos, robot.pos + (Vec(0.,700.) * robot.heading), 200.);
  bool isFree = true;
  for (unsigned int i=0; i < MWM.get_obstacle_location(t).size(); i++) {
    if (inFront.is_inside(MWM.get_obstacle_location(t)[i].pos)) {
      isFree = false;
      break;
    }
  }
    
  if (cycles_on_target > 5 && goal_line_points.size() > 0 && isFree) {
    Vec goal_line_point = goal_line_points[0];
  
      LOUT << "% dark_blue solid cross " << goal_line_point << "\n";
      double distance = (robot.pos - goal_line_point).length();
    bool reachable = false;

      // GESCHWINDIGKEITSHACK: Wenn weit weg, volle Staerke. Ansonsten, wenn der Roboter faehrt, nur halbe Staerke)
      if (distance > 6000.) {
        dv.klength = 60;
      }
      else {
        dv.klength = WBOARD->getKickLength(distance, 800., &reachable);
      }
      if(!reachable) {
        LOUT << "try to shoot, but WBOARD told me that I cannot reach the target";
      }
    if (dv.klength < 30) {
      dv.klength = 30;
      
    }
    cycles_on_target = 0;
      
  dv.kick = 1;
  }
  return dv;
}
  
  
void 
BEindhoven::gainControl(const Time& t) 
  throw(TribotsException)
{
  cycles_on_target = 0;
  skill->gainControl(t);
}

void 
BEindhoven::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
}
    

};
