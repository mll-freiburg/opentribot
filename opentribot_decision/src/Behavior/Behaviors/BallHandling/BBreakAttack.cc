#include "BBreakAttack.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include <cmath>
#include <iostream>

namespace Tribots
{
using namespace std;

BBreakAttack::
BBreakAttack(double transVel)
  : Behavior("BBreakAttack"), 
    transVel(transVel),
    skill(new SDribbleBallToPos()),
    swing(false)
{}

BBreakAttack::
~BBreakAttack() throw ()
{
  delete skill;
}

void BBreakAttack::updateTactics (const TacticsBoard& tb) 
    throw () 
{
    delete skill;
    skill = new SDribbleBallToPos();

  // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
        transVel= 1.4;
  } else	  
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
       transVel= 1.0;
  } else
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
       transVel= 2.3;
  } else { // =normal
       transVel= 2.0;
  }

  LOUT << "In BBreakAttack tactics changed. New skill: " 
       << skill->getName() << ". transVel=" << transVel << "\n";
}

bool 
BBreakAttack::
checkCommitmentCondition(const Time& t) throw()
{
  if (! WBOARD->doPossessBall(t)) {
    return false;
  }
  if (MWM.get_robot_location(t).pos.y < -MWM.get_field_geometry().field_length / 5.)  {
    return false;
  }
  return true;    
}

bool 
BBreakAttack::
checkInvocationCondition(const Time& t) throw()
{
  if (! BBreakAttack::checkCommitmentCondition(t)) {
    return false;
  }
  const FieldGeometry& field = MWM.get_field_geometry();
  XYRectangle rect(Vec(-field.penalty_area_width/2., field.field_length/2. - field.penalty_area_length - 1000.),
                   Vec(+field.penalty_area_width/2., field.field_length/2. - field.penalty_area_length / 2.));
  LOUT << "% blue solid " << rect << endl;
  int numObst = 0;
  for (unsigned int i=0; i < MWM.get_obstacle_location(t).size(); i++) {
    if (rect.is_inside(MWM.get_obstacle_location(t)[i].pos)) numObst++;
  }
  if (numObst < 2) {
    return false;
  }
  return true;
}

DriveVector 
BBreakAttack::getCmd(const Time& t) 
  throw(TribotsException) 
{
  FieldGeometry const& fgeom= MWM.get_field_geometry();
  Vec targetPos(0., -fgeom.field_length / 2.);  
  
  // \TODO : gets the "unfiltered" robot location 
  Time now;
  RobotLocation robot = MWM.get_slfilter_robot_location(now); // achtung: diese methode schreibt in now!
  
  bool forceHardTurn = 0;

  if (!WBOARD->isFreeDribbleCorridor(targetPos, t)) {
    if (swing) {
      LOUT << "% orange thin dotted cross " << targetPos << endl;
      if (fabs(robot.pos.x) < fgeom.field_width/2.-2250.) {
        swing = false;
      }
      if ((targetPos-robot.pos).angle(Vec(0.,1)*robot.heading).get_deg_180() < 0 && 
           swingDir == SWING_RIGHT) {
        targetPos = robot.pos + 
          (Vec(0.,2000.)*robot.heading).rotate(-Angle::three_eighth);
        LOUT << "siwng right" << endl;
      }
      else if ((targetPos-robot.pos).angle(Vec(0.,1)*robot.heading).get_deg_180()> 0 && 
               swingDir == SWING_LEFT) {
        targetPos = robot.pos + 
          (Vec(0.,2000.)*robot.heading).rotate(Angle::three_eighth);
        LOUT << "swing left" << endl;
      }
      else { // ausreichend weit
        LOUT << "swing weit genug gedreht, nun Punkt zur Mitte" << endl;
        targetPos = robot.pos + (targetPos-robot.pos).rotate(
          swingDir == SWING_LEFT ? -Angle::quarter : Angle::quarter).normalize() * 2000.;
      }
    }
    else {
      Vec origTarget = targetPos;
      targetPos = WBOARD->getEvasiveManeuverTarget(targetPos, t);
      if (fabs(targetPos.x) > fgeom.field_width/2. &&
          fabs(robot.pos.x) > fgeom.field_width/2. - 2000.) {
        LOUT << "Zielposition im Aus" << endl;
        if ((robot.pos.x > 0 ? -1. : 1.) * 
            Vec(0.,1.).angle(Vec(0.,1.)*robot.heading).get_deg_180() > 45) {
          LOUT << "Beginne im naechsten Zyklus mit Haken" << endl;
          swing = true;
          swingDir = (origTarget-robot.pos).angle(targetPos-robot.pos).get_deg_180() < 0 ? SWING_RIGHT : SWING_LEFT;
        }
        else {  // schaut nicht wirklich nach außen, daher nach innen ausweichen
          LOUT << "Kein Haken, schaut nicht ins aus, daher nach Innen ausweichen" 
               << endl;
          LOUT << "% yellow solid thin cross " << targetPos << endl;
          targetPos = robot.pos + (origTarget-robot.pos).rotate(
            targetPos.x > 0 ? Angle::quarter : -Angle::quarter).normalize()*2000.;
          LOUT << "% orange solid thin cross " << targetPos << endl;
          LOUT << "% orange solid thin word " << targetPos + Vec(100.,0.) 
               << "innen" << endl;
        }
      }
    }
  }
  
  skill->setParameters(targetPos, transVel, forceHardTurn);
  return skill->getCmd(t);
}
  
  
void 
BBreakAttack::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
}

void 
BBreakAttack::loseControl(const Time& t) 
  throw(TribotsException)
{
  swing = false;
  skill->loseControl(t);
}
    

};
