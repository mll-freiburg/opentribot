#include "BWingAttack.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/random.h"
#include "../../Predicates/freeCorridor.h"
#include "../../Skills/BallHandling/SDribbleBallStraightToPosEvadeSidewards.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"


#include <cmath>
#include <iostream>

// Implementierungshinweis:
// Es gibt zwei Phasen in diesem Verhalten:
//   1. Fahren in Richtung der Seitenlinie in einem Winkel < 45 Grad. Hierzu
//      wird das Skill mit einer Position schräg nach außen vor dem Roboter
//      aufgerufen. Ist der Roboter nahe am Außenbereich (ca 1.5m) dann wird
//      auf 2. Umgeschaltet
//   2. Anfahrt der Zielposition (targetPoint) im Bereich von BEigenMove. Dies
//      geschieht solange, bis ein höheres Verhalten übernimmt oder das Ziel
//      erreicht wird.
// Das Verhalten gibt die Kontrolle ab, wenn in Fahrtrichtung ein Hindernis
// auftaucht oder der Weg zumm Tor komplett frei ist (optional) oder der 
// Zielpunkt erreicht ist oder der Roboter nicht mehr nach vorne schaut oder
// der Ball mehr als 10 Zyklen verloren wurde.

namespace Tribots
{
  
using namespace std;

BWingAttack::
BWingAttack(double transVel, double activationProbability, bool earlyExit)
  : Behavior("BWingAttack"), 
    transVel(transVel), 
    activationProbability(activationProbability),
    earlyExit(earlyExit),
    boosting(true),
    useEvade(true),
    skill(new SDribbleBallToPos()),
    skillEvade(new SDribbleBallStraightToPosEvadeSidewards(1200.)),
    fast_skill(new SBoostToPos(1500 , true , 0.2)),
    activeSkill(0)
{
  m_bDribblingToSecondPoint =false;
  reached = false;
  m_bDecisionMade = false;
  m_iLostBallLoops = 0;
 
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  // Zielpunkt im Aktivierungsbereich von BEigenMove, wird für linke 
  // Spielfeldhälfte gespiegelt
  targetPoint = Vec((fgeom.field_width/2.  - 1200.), 
                     fgeom.field_length/2. - 2000);
}


BWingAttack::~BWingAttack() throw ()
{
  delete skill;
  delete skillEvade;
  delete fast_skill;
}

void 
BWingAttack::cycleCallBack (const Time& t) throw()
{
  if (! WBOARD->doPossessBall(t)) {
    if (m_iLostBallLoops>10) {
      m_bDecisionMade = false;
    }
    m_iLostBallLoops++;
  }
  else if (m_bDecisionMade == false) {
    mayBecomeActive = brandom(activationProbability);
    m_bDecisionMade = true;
    m_iLostBallLoops = 0;
  }
}

bool 
BWingAttack::checkCommitmentCondition(const Time& t) throw()
{
  if (!WBOARD->doPossessBall(t) && m_iLostBallLoops>10) {
    return false;
  }
  
  if (reached) {
    return false;
  }
  
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  if (m_bDribblingToSecondPoint &&   // schaut zu weit weg
      (robot.heading.get_deg_180() > 90 ||
       robot.heading.get_deg_180() < -90)) {
    return false;
  }
  
  if (m_bDribblingToSecondPoint && 
      (Vec(targetPoint.x*(robot.pos.x>0?1:-1),
           targetPoint.y)-robot.pos).length() < 700.) {
    return false;                    // nah genug, aufhören 
  }
  
  // vorne frei? andernfalls sofort rausspringen, wenn nicht evade sidewards
  // verwendet wird. bei evade sidewards nur bei "blockade" rausspringen!
  double distance = 
    obstacle_distance_to_line_inside_field(robot.pos, robot.pos + Vec(0., 1.).rotate(robot.heading).normalize()*(useEvade?700.:2500.), MWM.get_obstacle_location(t), true);
  if (distance < (useEvade?200.:400.)) { // ziemlich enger korridor....
    return false;          // bei evade musser sogar fast auf der linie sein
  }
  
  // fruehen ausstieg checken, ACHTUNG: immer als letzten check, da semantik
  //                                    verdreht (returns true statt false)
  if (earlyExit) {
    if (robot.pos.y < 0. || robot.pos.y > fgeom.field_length/2. - 2000.) {
      return true;
    }
    double sign = robot.pos.x > 0 ? 1 : -1;
    Quadrangle area(robot.pos - Vec(750.,0) * sign, 
                    Vec(0., fgeom.field_length/2.-1000),
                    Vec(robot.pos.x + 300 * sign, fgeom.field_length/2.-500),
                    Vec((fgeom.field_width/2.-200) * sign, robot.pos.y));
    LOUT << "\% grey solid " << area << endl;
    
    for (unsigned int i=0; i < MWM.get_obstacle_location(t).size(); i++) {
      if (area.is_inside(MWM.get_obstacle_location(t)[i].pos) && MWM.get_obstacle_location(t)[i].player<0) {
        return true;
      }
    }
    return false; // aufhoeren, voellig freie Bahn zum Tor    
  }
  return true;    
}

bool 
BWingAttack::checkInvocationCondition(const Time& t) throw()
{
  if (!WBOARD->doPossessBall(t)) {
    return false;
  }
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  // im Bereich, in dem angefangen werden darf (eigene hälfte, in nähe der
  // seitenlinien)?
  if (!mayBecomeActive || robot.pos.y > 0 ||      // nicht im vorfeld, nicht
      fabs(robot.pos.x) < fgeom.field_width/4.) { // in der mitte des feldes
    return false;
  }
  // Blickrichtung checken
  double heading = robot.heading.get_deg_180();
  if (robot.pos.x > 0) {
    if (robot.pos.x < fgeom.field_width/2.-1000. &&
        !(heading < 0 && heading > -90)) {
      return false;
    }
    else if (robot.pos.x > fgeom.field_width/2.-1000. &&
             !(heading < 90 && heading > -90)) {
      return false;
    }
  }
  else {
    if (robot.pos.x > -fgeom.field_width/2.+1000. &&
        !(heading < 90 && heading > 0)) {
      return false;
    }
    else if (robot.pos.x < -fgeom.field_width/2.+1000. &&
             !(heading < 90 && heading > -90)) {
      return false;
    }
  }
  // Freier Platz in Blickrichtung
  double distance = obstacle_distance_to_line_inside_field(robot.pos, robot.pos + Vec(0., 1.).rotate(robot.heading).normalize()*1000., MWM.get_obstacle_location(t),true);
  if (distance < 500.) { // zu wenig Platz
    return false;
  }
  // Freier Platz in Zielrichtung
  Vec targetPos((fgeom.field_width/2. - 1000.) * (robot.pos.x > 0 ? 1 : -1),
                robot.pos.y);
  targetPos = targetPos + Vec(0, fabs(robot.pos.x - targetPos.x) * 2.); // flacher winkel richtung seitenlinie
  distance = obstacle_distance_to_line_inside_field(robot.pos, targetPos, MWM.get_obstacle_location(t));
  if (distance < 500.) {
    return false;
  }  
  
  return true;  
}

DriveVector BWingAttack::getCmd(const Time& t) throw(TribotsException) 
{
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  const RobotLocation& robot = MWM.get_robot_location(t);

  if (fabs(robot.pos.x) > fgeom.field_width/6. ||
      robot.pos.y > 1000.) {
    m_bDribblingToSecondPoint = true;
  }
  
  Vec targetPos;
  Skill* newSkill;

  // TODO: gainControl und loseControl an die Skills geben!
  if (m_bDribblingToSecondPoint) { // in 2. Phase, Zielpunkt andribbeln
    targetPos = Vec(targetPoint.x * (robot.pos.x > 0 ? 1 : -1), targetPoint.y); 
    fast_skill->setTargetPos(targetPos);
    if (boosting && fast_skill->checkInvocationCondition(t)) {    
      newSkill = fast_skill;
      LOUT << "use boost" << endl;
    } else if (!useEvade || 
               fabs((((targetPos-robot.pos)/robot.heading).angle()-      
                     Angle::quarter).get_deg_180()) > 30) {
      skill->setParameters(targetPos, transVel, false);
      newSkill = skill;
      LOUT << "use gotopos" << endl;
    }
    else { // useEvade
      skillEvade->setTransVel(transVel);
      skillEvade->setTarget(targetPos, targetPos);
      newSkill = skillEvade;
      LOUT << "use evade sidewards" << endl;
    }
  }
  else {  
    targetPos = Vec((fgeom.field_width/2. - 1000.) * (robot.pos.x > 0 ? 1 : -1),
                    robot.pos.y);
    targetPos = targetPos + Vec(0, fabs(robot.pos.x - targetPos.x) * 2.); // flacher winkel
    skill->setParameters(targetPos, transVel, false);
    newSkill = skill;
    LOUT << "use gotopos" << endl;
  }

  if (newSkill != activeSkill) {
    if (activeSkill) activeSkill->loseControl(t);
    activeSkill=newSkill;
    activeSkill->gainControl(t);
  }
  return activeSkill->getCmd(t);
}
  
  
void 
BWingAttack::gainControl(const Time& t) 
  throw(TribotsException)
{
  activeSkill=0;
}

void 
BWingAttack::loseControl(const Time& t) 
  throw(TribotsException)
{
  activeSkill->loseControl(t);
  m_bDribblingToSecondPoint=false;
  reached = false;
}

void BWingAttack::updateTactics (const TacticsBoard& tb) throw ()
{


    skill = new SDribbleBallToPos();

  // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
        transVel=1.9;
  } else      
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
       transVel= 1.2;
  } else
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
       transVel= 2.6;
  } else { // =normal
       transVel= 2.1;
  }

  string key = "Fluegelangriff";
  if (tb[key] == string("nie")) {
    activationProbability = 0.;
	}
  else if (tb[key] == string("selten")) {
    activationProbability = .25;
  }
  else if (tb[key] == string("manchmal")) {
    activationProbability = .5;
  }
  else if (tb[key] == string("oft")) {
    activationProbability = .75;
  }
  else {
    activationProbability = 1.;
  }
  
  if (tb[string("FluegelangriffAusstieg")] == string("ja")) {
    earlyExit = true;
  }
  else {
    earlyExit = false;
  }
  
  if (tb[string("FluegelangriffSeitwaerts")] == string("ja")) {
    useEvade = true;
  }
  else {
    useEvade = false;
  }
  
  if (tb[string("boosting")] == string("nein")) {
    boosting = false;
  }
  else {
    boosting = true;
  }


  LOUT << "BWingAttack Taktikwechsel prob=" << activationProbability
       << " frueher Ausstieg moeglich: " << (earlyExit?"ja":"nein") 
       << " Geschwindigkeit: " << transVel  
       << " boosting: " << boosting 
       << " normaler Dribbleskill: " << skill->getName() << endl;
}


};
