#include "BShakeOffDefender.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include "../../Predicates/freeCorridor.h"
#include <cmath>
#include <iostream>

namespace Tribots
{
using namespace std;

BShakeOffDefender::
BShakeOffDefender(double transVel)
  : Behavior("BShakeOffDefender"), 
    activated(true),
    transVel(transVel),
    skill(new SDribbleBallToPos()),
    swingReverse(false), haveObst(false),
    doNotActivateForHighSpeeds(false)
{}

BShakeOffDefender::
~BShakeOffDefender() throw ()
{
  delete skill;
}

void BShakeOffDefender::updateTactics (const TacticsBoard& tb) 
    throw () 
{
    delete skill;
    skill = new SDribbleBallToPos();
  
  // SchlenkerMoves = alleAus bolzerGarNicht *bolzerNichtAbschuetteln alleAn
  if (tb[string("SchlenkerMoves")] == "alleAus") {
    activated = false;
  } else if (tb[string("SchlenkerMoves")] == "alleAn") {
    activated = true;
  } else { // =normal
    activated = MWM.get_robot_properties().kickers <= 1;
  }

  // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
        transVel= 1.4;
  } else	  
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
       transVel=1.0;
  } else
  if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
       transVel= 2.3;
  } else { // =normal
       transVel= 2.0;
  }

  LOUT << "In BShakeOffDefender tactics changed. New skill: " 
       << skill->getName() << ". transVel=" << transVel << "\n";
}

bool 
BShakeOffDefender::
checkCommitmentCondition(const Time& t) throw()
{
  if (state == SOD_DONE) {
    LOUT << "BShakeOffDefender::checkCOmmitmentCondition(): Manoever erfolgreich abgeschlossen. Hoere deshalb auf." << endl;
    return false;
  }
  if (! WBOARD->doPossessBall(t)) {
    return false;
  }
  if (MWM.get_robot_location(t).pos.y < -MWM.get_field_geometry().field_length / 5.)  {
    return false;
  }
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& field = MWM.get_field_geometry();
  if (robot.pos.y > field.field_length/2. - field.penalty_area_length+500.) { // im strafraum ist dieses Verhalten nicht mehr sinnvoll
    LOUT << "Roboter im Strafraum. Breche BShakeOffDefender ab." << endl;
    return false;
  }
  if (haveObst) {
    Quadrangle robotFront(robot.pos + Vec(0,300.) * robot.heading, robot.pos + Vec(0,600.) * robot.heading, 400);
    if (robotFront.is_inside(lastObstPos)) {   // der weg ist auf das uebelste blockiert, es geht mit diesem Skill nicht weiter.
      LOUT << "% red solid " << robotFront << endl;
      LOUT << "Hindernis direkt vor Roboter. Breche BShakeOffDefender ab." << endl;
      return false;
    }
  }
  return true;    
}

bool 
BShakeOffDefender::
checkInvocationCondition(const Time& t) throw()
{
  if (!activated) {
    return false;
  }
  if (!WBOARD->doPossessBall(t)) {
    return false;
  }
  if (state == SOD_DONE) {
    return false;
  }
  const RobotLocation& robot = MWM.get_robot_location(t);
  const FieldGeometry& field = MWM.get_field_geometry();
  if (doNotActivateForHighSpeeds && robot.vtrans.length() > 2.0) {
    return false;
  }
  if (fabs(robot.heading.get_deg_180()) > 30) { // schaut in richtung gegenerische Grundlinie
    return false;
  }
  vector<Vec> results = intersect(LineSegment(Vec(-field.field_width/2.,field.field_length/2.), 
                                              Vec(+field.field_width/2.,field.field_length/2.)),
                                  LineSegment(robot.pos, robot.pos + Vec(0.,field.field_length) * robot.heading));
  if (results.size() == 0 ||                                    // schneidet die Fahrtrichtung des Roboters die Grundlinie?
      fabs((robot.vtrans / Angle::quarter).angle().get_deg_180()) > 60) { // und faehrt er noch nicht so stark zur Seite?
    return false;
  }
  if (fabs(robot.pos.x) > field.penalty_area_width / 2. + 500 ||    // Roboter befindet sich vor der PenaltyArea
      robot.pos.y < 0 || robot.pos.y > field.field_length/2. - field.penalty_area_length - 1000.) {
    return false;
  }

  // Schauen, ob ausweichen noetig ist
  if (!haveObst) {
    return false;
  }
  // Schauen, ob ausweichen mit diesem Verhalten noch moeglich ist
  if ((robot.pos - lastObstPos).length() < 700.) { // zu Nah dran!
    return false;
  }
  return true;
}

void 
BShakeOffDefender::cycleCallBack(const Time& t) throw()
{
  
  const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
  // Hindernisse anschauen
  const RobotLocation& robot = MWM.get_robot_location(t);
  vector<Vec> relevantObstacles;
  double lookAheadDistance = robot.vtrans.length() > 2.0 && !doNotActivateForHighSpeeds ? 2400. : 1800.;
  Quadrangle inFrontOfRobot(robot.pos + Vec(0,100.) * robot.heading, robot.pos + Vec(0,lookAheadDistance) * robot.heading, 500);
  Quadrangle inWayOfRobot(robot.pos + (robot.vtrans.normalize() * 100.), robot.pos + robot.vtrans.normalize() * doNotActivateForHighSpeeds, 500);
  LOUT << "% blue thin dotted " << inFrontOfRobot << endl;
  LOUT << "% blue thin solid " << inWayOfRobot << endl;
  for (unsigned int i=0; i < MWM.get_obstacle_location(t).size(); i++) {
    if (inFrontOfRobot.is_inside(MWM.get_obstacle_location(t)[i].pos) || inWayOfRobot.is_inside(MWM.get_obstacle_location(t)[i].pos)) {
      LOUT << "% red thin solid circle " << Circle(MWM.get_obstacle_location(t)[i].pos, 500.) << endl;
      relevantObstacles.push_back(MWM.get_obstacle_location(t)[i].pos);
    }
  }
  switch (state) {
  case SOD_OFF:
      if (relevantObstacles.size() == 0) {
        haveObst = false;
      }
      else if (relevantObstacles.size() == 1) {
        haveObst = true;
        lastObstPos = relevantObstacles[0];
      }
      else {
        int minObst = 0;
        for (unsigned int i=1; i < relevantObstacles.size(); i++) {
          if ((robot.pos - relevantObstacles[i]).length() < 
              (robot.pos - relevantObstacles[minObst]).length()) {
            minObst = i;
          }
        }
        haveObst = true;
        lastObstPos = relevantObstacles[minObst];
      }
    break;
  default:
      bool hadObst = haveObst;
      haveObst = false;
      // erstmal checken, ob das alte hindernis wiedergefunden werden kann
      if (hadObst && MWM.get_obstacle_location(t).size() > 0) {
        int minObst = 0;
        for (unsigned int i=1; i < MWM.get_obstacle_location(t).size(); i++) {
          if ((lastObstPos - MWM.get_obstacle_location(t)[i].pos).length() < (lastObstPos - MWM.get_obstacle_location(t)[minObst].pos).length()) {
            minObst = i;
          }
        }
        if ((lastObstPos-obstacles[minObst].pos).length() < 500.) {
          lastObstPos = obstacles[minObst].pos; // wahrscheinlich noch dasselbe, also updaten
          haveObst = true;
        }
      }
      // jetzt das nahegelegenste aus den relevanten raussuchen
      if (relevantObstacles.size() > 0) {
        int minObst = 0;
        for (unsigned int i=1; i < relevantObstacles.size(); i++) {
          if ((robot.pos - relevantObstacles[i]).length() < 
              (robot.pos - relevantObstacles[minObst]).length()) {
            minObst = i;
          }
        }
        if (!haveObst) {  // das alte nicht gefunden, oder keins gehabt
          haveObst = true;
          lastObstPos = relevantObstacles[minObst];
        }
        else {            // mit dem aktualisierten alten hindernis vergleichen und entscheiden, ob ein switch sinnvoll ist
          if ((relevantObstacles[minObst]-lastObstPos).length() > 1.) { // das ist wohl nicht das gleiche hindernis
            Quadrangle robotFront(robot.pos, robot.pos + Vec(0, 700.), 500);
            if (robotFront.is_inside(relevantObstacles[minObst])) {
              lastObstPos = relevantObstacles[minObst];
              haveObst = true;  // nujo, doppelt gemoppelt, aber der logik halber, weil es hier auch gesetzt wird
            }
          }
        }
      }
      break;
  }
  if (haveObst) {
    LOUT << "% yellow solid " << Circle(lastObstPos, 600) << endl;
  }
}
      
    

DriveVector 
BShakeOffDefender::getCmd(const Time& t) 
  throw(TribotsException) 
{
  const FieldGeometry& field= MWM.get_field_geometry();
  const RobotLocation& robot = MWM.get_robot_location(t);   

  DriveVector dv;
  string stateDesc[5] = {"OFF", "TURN", "DRIVE", "TURN_BACK", "DONE" };
  LOUT << "BShakeOffDefender::getCmd(): present state is: " << stateDesc[state] << endl;
  
  switch (state) {
  case SOD_TURN: 
  {
    double targetAngle = swingDir == SWING_LEFT ? 60 : -60;
    if (swingReverse) { // die grosse runde drehen (> 180 grad). dazu langsam den zielwinkel bis zur neuen Richtung anpassen
      double presentAngle = robot.heading.get_deg_180();
      double presentTargetAngle = presentAngle - (targetAngle/fabs(targetAngle)) * 60;
      double tmpAngle = Angle::deg_angle(presentTargetAngle).get_deg_180();
      if (swingDir == SWING_LEFT && !(tmpAngle < targetAngle && tmpAngle >= -10)) { // drehe links rum && ! (weit genug)
        targetAngle = presentTargetAngle; // Winkel langsam anpassen
      }
      if (swingDir == SWING_RIGHT && !(tmpAngle > targetAngle && tmpAngle <= 10)) {
        targetAngle = presentTargetAngle;
      }
    }        
      
    Vec targetPos = robot.pos + Vec(0, 3000.) * Angle::deg_angle(targetAngle);
    skill->setParameters(targetPos, transVel, false);
    dv = skill->getCmd(t);
    LOUT << "% yellow solid thick " << Circle (targetPos, 500) << endl;
    
    if (fabs((robot.heading-Angle::deg_angle(targetAngle)).get_deg_180()) < 7.5) {
      state = SOD_DRIVE;
      tmpPos = robot.pos; // neuer Zustand, Position zu Beginn merken, um spaeter schauen zu koennen, wie weit bereits geradeaus gefahren wurde.
    }
    break;
  }
  case SOD_DRIVE:
  {
    Angle targetAngle = (swingDir == SWING_LEFT ? Angle::deg_angle(60) : Angle::deg_angle(-60));
    Vec targetPos = robot.pos + Vec(0, 3000.) * targetAngle;
    skill->setParameters(targetPos, transVel, false);
    dv = skill->getCmd(t);
    
    bool isFreeToGoal = false;
    if ((!haveObst || (fabs(lastObstPos.x - robot.pos.x) > 700. || lastObstPos.y + 300. < robot.pos.y)) && // relevantes Hindernis keine Gefahr mehr 
        obstacle_distance_to_line(robot.pos, robot.pos + 
                                  (Vec(0,field.field_length/2.) - robot.pos).normalize() * 1500, MWM.get_obstacle_location(t)) > 400) {  // und Weg Richtung Tor relativ frei   
      isFreeToGoal = true;
    }
    if (isFreeToGoal) {
      state = SOD_TURN_BACK;
    }
    else {
      if (fabs(tmpPos.x-robot.pos.x) > 1200.) { // 1.2m seitlich zurueckgelegt und immer noch nicht frei -> Drehung in andere Richtung versuchen
        state = SOD_TURN;
        swingDir = swingDir == SWING_LEFT ? SWING_RIGHT : SWING_LEFT;
        swingReverse = true;
        tmpPos = robot.pos; // neuer Zustand begonnen, Position merken
      }
    }
    break;
  }
  case SOD_TURN_BACK:
  {
    Vec targetPos(0, field.field_length/2.);
    skill->setParameters(targetPos, transVel, false);
    dv = skill->getCmd(t);    
    
    vector<Vec> results = intersect(LineSegment(Vec(-field.goal_width / 2., field.field_length/2.),
                                                Vec(+field.goal_width / 2., field.field_length/2.)),
                                    LineSegment(robot.pos, robot.pos + Vec(0,field.field_length) * robot.heading));
    if (results.size() > 0) {
      state = SOD_DONE;
    }
    break;
    // Idee: Man koennte noch schauen, ob das Hindernis beim Zurueckdrehen wieder in den fahrtweg kommt. wenn ja, dann einfach noch mal zu TURN umschalten, ohne die Richtung (swingDir) zu aendern, reverseDir in dem Fall = false. Wuerde dann eine Treppe / Saegezaehne ergeben...
  }
  default:  // SOD_DONE
  {
    dv = skill->getCmd(t);
  }
  }
  return dv;
}
  
  
void 
BShakeOffDefender::gainControl(const Time& t) 
  throw(TribotsException)
{
  const RobotLocation& robot = MWM.get_robot_location(t);
  skill->gainControl(t);
  state = SOD_TURN;
  tmpPos = robot.pos;
  // jetzt entscheiden, in welche Richtung ausgewichen werden soll
  if (robot.pos.x < 0) {
    swingDir = SWING_RIGHT;
    if (haveObst && lastObstPos.x - 300 > robot.pos.x) {
      swingDir = SWING_LEFT;
    }
  }
  if (robot.pos.x > 0) {
    swingDir = SWING_LEFT;
    if (haveObst && lastObstPos.x + 300. < robot.pos.x) {
      swingDir = SWING_RIGHT;
    }
  }
  swingReverse = false;
}

void 
BShakeOffDefender::loseControl(const Time& t) 
  throw(TribotsException)
{
  skill->loseControl(t);
  state = SOD_OFF;
}
    

};
