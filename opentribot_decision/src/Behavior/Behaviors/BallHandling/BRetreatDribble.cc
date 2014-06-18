#include "BRetreatDribble.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include "../../Predicates/freeCorridor.h"
#include "../../../Fundamental/random.h"

#include <cmath>
#include <iostream>

namespace Tribots
{
using namespace std;

BRetreatDribble::BRetreatDribble(double transVel)
  : Behavior("BRetreatDribble"), 
    activationLevel(ON),
    transVel(transVel),
    skill(new SDribbleBallToPos()),
    swing(false), reverseTurn(false),
    mayBecomeActive(false),
    m_bDecisionMade(false),
    m_iLostBallLoops(0)
{}

BRetreatDribble::~BRetreatDribble() throw ()
{
  delete skill;
}
  
void BRetreatDribble::cycleCallBack(const Time& t) throw()
{
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& field = MWM.get_field_geometry();
  if (!WBOARD->doPossessBall(t) || robot.pos.y < 0) {
    positions.clear();
  }
  else if (positions.size() > 0 && positions[0].x * robot.pos.x < 0) {
    positions.clear();
  }
  else if (!(positions.size() == 0 && fabs(robot.pos.x) > field.penalty_area_width/2. + 1000.))   { // nicht ausserhalb des strafraums damit anfangen, die positionen aufzuzeichnen
    positions.push_back(MWM.get_robot_location(t).pos);
  }
  
    if (! WBOARD->doPossessBall(t)) {
      if (m_iLostBallLoops>10) {
        m_bDecisionMade = false;
      }
      m_iLostBallLoops++;
    }
    else if (m_bDecisionMade == false) {
      mayBecomeActive = brandom(0.5);
      m_bDecisionMade = true;
      m_iLostBallLoops = 0;
    }
    
  
}

void BRetreatDribble::updateTactics (const TacticsBoard& tb) 
    throw () 
{
    delete skill;
    skill = new SDribbleBallToPos();
  
  // SchlenkerMoves = alleAus bolzerGarNicht *bolzerNichtAbschuetteln alleAn
  if (tb[string("SchlenkerMoves")] == "alleAus") {
    activationLevel = OFF;
  } else if (tb[string("SchlenkerMoves")] == "alleAn") {
    activationLevel= ON;
  } else if (tb[string("SchlenkerMoves")] == "bolzerGarNicht") { // =normal
    activationLevel = MWM.get_robot_properties().kickers <= 1 ? ON : OFF;
  }
  else { // bolzerNichtAbschuetteln
    activationLevel = MWM.get_robot_properties().kickers <= 1 ? ON : ONLY_IN_PENALTY_AREA;    
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

  LOUT << "In BRetreatDribble tactics changed. New skill: " 
       << skill->getName() << ". transVel=" << transVel << "\n";
}


bool 
BRetreatDribble::
checkCommitmentCondition(const Time& t) throw()
{
  
  const RobotLocation& robot = MWM.get_robot_location(t);
  if (! WBOARD->doPossessBall(t)) {
    return false;
  }
  if (robot.pos.y < 0)  {
    return false;
  }
  if ((startPos.x > 0 && robot.pos.x < -500) ||
      (startPos.x < 0 && robot.pos.x > +500)) {
    return false;
  }
  return true;    
}

bool 
BRetreatDribble::
checkInvocationCondition(const Time& t) throw()
{
  if (activationLevel == OFF) {
    return false;
  }
  
  const RobotLocation& robot = MWM.get_robot_location(t);
  if (! WBOARD->doPossessBall(t) || robot.pos.y < 0) {
    return false;
  }
  const FieldGeometry& field = MWM.get_field_geometry();
  const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
  XYRectangle rect(Vec(field.goal_width/2., field.field_length/2),
                   Vec(field.penalty_area_width/2., field.field_length/2 - field.penalty_area_length));
  Vec fAbsPos = Vec(fabs(robot.pos.x), robot.pos.y);
  if (rect.is_inside(fAbsPos)) {  // erster Fall: im Strafraum, vor den Pfosten gelangt
    return true;
  }
  if (activationLevel == ONLY_IN_PENALTY_AREA || !mayBecomeActive) {  // do not activate outside the penalty area or if the random draw was negative
    return false;
  }
  if (fabs(robot.pos.x) < field.goal_width/2. || fabs(robot.pos.x) > field.penalty_area_width / 2. + 500.) { // noch vorm Tor oder zu weit aussen
    return false;
  }
  if (positions.size() < 30) {
    return false;
  }
  if (fabs(positions[positions.size()-30].x - positions[positions.size()-1].x) < 900. &&  // in letzter sekunde  sehr weit ausgewichen
      fabs(positions[0].x - positions[positions.size()-1].x) < 1500.) {  // oder insgesamt sehr weit ausgewichen
    return false;
  }
  LOUT << "% blue solid " << Line(robot.pos, robot.pos + Vec(0,1500.)) << endl;
  if (obstacle_distance_to_line(robot.pos, robot.pos + Vec(0,1500.) * robot.heading, obstacles, true) < 600.) {
    return true;
  }
  return false;
}

DriveVector 
BRetreatDribble::getCmd(const Time& t) 
  throw(TribotsException) 
{
  FieldGeometry const& fgeom= MWM.get_field_geometry();
  RobotLocation robot = MWM.get_robot_location(t); 
  Vec targetPos = target;
  if (reverseTurn) {
    targetPos = Vec((robot.pos.x < 0 ? - 2000. : + 2000.) + robot.pos.x, robot.pos.y - 500.);
    if (robot.heading.get_deg_180() > 90 && robot.pos.x < 0 || robot.heading.get_deg_180() < -90 && robot.pos.x > 0) {
      reverseTurn = false;
    }
  }
  
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
BRetreatDribble::gainControl(const Time& t) 
  throw(TribotsException)
{
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& field = MWM.get_field_geometry();
  startPos = robot.pos;
  target = Vec(startPos.x > 0 ? -1000. : 1000., startPos.y);
  if (target.y > field.field_length / 2. - field.penalty_area_length - 1500.) {
    vector<Vec> result = intersect(LineSegment(Vec(-field.goal_width/2., field.field_length/2.), Vec(+field.goal_width/2., field.field_length/2.)),
                                   LineSegment(robot.pos, robot.pos + Vec(0,field.field_length) * robot.heading)); 
    target.y = field.field_length / 2. - field.penalty_area_length - 1500;
    reverseTurn = robot.heading.get_deg_180() * robot.pos.x < 0. && result.size() == 0;
  }
  skill->gainControl(t);
}

void 
BRetreatDribble::loseControl(const Time& t) 
  throw(TribotsException)
{
  swing = false;
  skill->loseControl(t);
  positions.clear();
}
    

};
