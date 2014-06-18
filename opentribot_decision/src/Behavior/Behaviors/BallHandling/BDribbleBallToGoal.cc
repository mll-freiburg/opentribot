#include "BDribbleBallToGoal.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include <cmath>
#include <iostream>

namespace Tribots
{
using namespace std;

BDribbleBallToGoal::
BDribbleBallToGoal(double transVel)
  : Behavior("BDribbleBallToGoal"), 
    doDribbleToMiddleFirst(false), state(0), transVel(transVel),
    skill(new SDribbleBallToPos()),
    swing(false)
{}

BDribbleBallToGoal::
~BDribbleBallToGoal() throw ()
{
  delete skill;
}

void BDribbleBallToGoal::updateTactics (const TacticsBoard& tb) 
    throw () 
{

    skill = new SDribbleBallToPos();
  
  if (tb[string("ErstInDieMitteDribbeln")] == "an") {
    doDribbleToMiddleFirst = true;
  }
  else {
    doDribbleToMiddleFirst = false;
  }

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

  LOUT << "In BDribbleBallToGoal tactics changed. New skill: " 
       << skill->getName() << ". transVel=" << transVel << "\n";
}

bool 
BDribbleBallToGoal::
checkCommitmentCondition(const Time& t) throw()
{
  if (! WBOARD->doPossessBall(t)) {
    return false;
  }
  return true;    
}

bool 
BDribbleBallToGoal::
checkInvocationCondition(const Time& t) throw()
{
  return BDribbleBallToGoal::checkCommitmentCondition(t);
}

DriveVector 
BDribbleBallToGoal::getCmd(const Time& t) 
  throw(TribotsException) 
{
  FieldGeometry const& fgeom= MWM.get_field_geometry();
  Vec oppGoalPos(0., fgeom.field_length / 2.);  
  
  // \TODO : gets the "unfiltered" robot location 
  Time now;
  RobotLocation robot = MWM.get_slfilter_robot_location(now); // achtung: diese methode schreibt in now!
  
  Vec targetPos;
  bool forceHardTurn = 0;

  // Status umschallten
  if (state == 0 && doDribbleToMiddleFirst) {
    if (robot.pos.y > fgeom.field_length/9. ||
        fabs(robot.pos.x) < fgeom.center_circle_radius) {
      state = 1;
    }
  }
  
  // targetPos berechnen
  if (state == 0 && doDribbleToMiddleFirst) {
    targetPos = Vec(0,0);
    if (targetPos.y - 2500. < robot.pos.y) {
      targetPos.y = robot.pos.y + 2500.;
    }
  }
  else {
    if ((robot.pos-oppGoalPos).length()>3500.) { // Abstand zum Tor > 3000?
      if (robot.pos.x > 0) {    // true=right=1, false=left=0
          targetPos = oppGoalPos+Vec(600.,0.); // rechter Torpfosten
      }
      else {
        targetPos = oppGoalPos+Vec(-600.,0.);// linker Torpfosten
      }
    }
    else {
      targetPos = WBOARD->getFreeGoalPos(t);
    }
  }

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
        else {  // schaut nicht wirklich nach au§en, daher nach innen ausweichen
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
BDribbleBallToGoal::gainControl(const Time& t) 
  throw(TribotsException)
{
  skill->gainControl(t);
  state = 0;
}

void 
BDribbleBallToGoal::loseControl(const Time& t) 
  throw(TribotsException)
{
  swing = false;
  skill->loseControl(t);
}
    

};
