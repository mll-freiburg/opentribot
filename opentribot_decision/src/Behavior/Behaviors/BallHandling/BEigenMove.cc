#include "BEigenMove.h"
#include "../../../Fundamental/geometry.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../Skills/BallHandling/SDribbleBallToPos.h"
#include <iostream>
#include <cmath>

namespace Tribots
{
  using namespace std;

  BEigenMove::BEigenMove(Vec targetPosRelToOppGoal) 
    : Behavior("BEigenMove"), skill(new SDribbleBallToPos())
  {
    targetPos = Vec(0, 0);// wird abhaengig von robot.pos in gaincontrol gesetzt
    transVel = 2.0;
  }

  BEigenMove::~BEigenMove() throw()
  {
    delete skill;
  }

  void BEigenMove::updateTactics (const TacticsBoard& tb) 
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

    LOUT << "In BEigenMove tactics changed. New skill: " 
         << skill->getName() << ". transVel=" << transVel << "\n";
  }

  void 
  BEigenMove::gainControl(const Time& t) throw(TribotsException)
  {
    const FieldGeometry& field =
      MWM.get_field_geometry();   
    
    /*
    if (MWM.get_robot_location(t).pos.y < field.field_length / 2. - 800.) {
      targetPos = Vec(0., MWM.get_robot_location(t).pos.y-2000.);
    }
    else if (MWM.get_robot_location(t).pos.x > 0) { // in der Naehe der Ecken
      targetPos = Vec(field.field_width/8., field.field_length/6.);
    }
    else {
      targetPos = Vec(-field.field_width/8., field.field_length/6.);
    } */
    // WM2008: wieder einen groe§eren Bogen bis in die Mitte des Spielfeldes machen.
    targetPos = Vec(0, (field.field_length/2.-field.penalty_area_length)/2.);
  }

  DriveVector
  BEigenMove::getCmd(const Time& t) throw(TribotsException)
  {
    Vec dribblePos = targetPos;

    LOUT << "\% grey " << Circle(dribblePos, 150.) << endl;
    if (! WBOARD->isFreeDribbleCorridor(dribblePos, t) ) {
      dribblePos = WBOARD->getEvasiveManeuverTarget(dribblePos, t);
    }

    LOUT << "\% red " << Circle(dribblePos, 200.) << endl;
    skill->setParameters(dribblePos, transVel, false);
    return skill->getCmd(t);
  }

  bool BEigenMove::checkInvocationCondition(const Time& t) throw()
  {
    Vec ballPos =
      MWM.get_ball_location(t).pos.toVec();
    const FieldGeometry& field =
      MWM.get_field_geometry();

    if (fabs(ballPos.x) < field.penalty_area_width/2. + 200. && 
        ballPos.y < field.field_length/2 - field.penalty_area_length/2.) {
      return false; // nicht beginnen, wenn ball weder am spielfeldrand noch
    }               // in naehe der grundlinie
    if (ballPos.y < field.field_length/2 - field.penalty_area_length * 1.5) {
      return false; // auch in randnaehe nicht zu weit weg vom Tor beginnen
    }
    if (fabs(ballPos.x) < field.goal_area_width/2.+500.) { 
      return false; // auch in grundliniennaehe nicht direkt vorm tor abdrehen
    }
    if (! WhiteBoard::getTheWhiteBoard()->doPossessBall(t)) {
      return false;
    }
    return true;
  }

  bool BEigenMove::checkCommitmentCondition(const Time& t) throw()
  {

    const FieldGeometry& field =
      MWM.get_field_geometry();
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    const RobotLocation& robot = MWM.get_robot_location(t);
    Vec robotPos=robot.pos;
    Angle robotHeading=robot.heading;
      
      
    if ((targetPos-robotPos).length() < 500) {
      return false;
    }
    if (!WhiteBoard::getTheWhiteBoard()->doPossessBall(t)) {
      return false;
    }
    
    Vec toGoal(Vec(0, field.field_length/2) - robotPos);
    Quadrangle towardsGoal(robotPos, robotPos + (toGoal.normalize() * 1500.), 
                           1200., 500.);
    Quadrangle forward(robotPos, robotPos + (Vec(0,1200.) * robotHeading), 
                       800., 1500.);
    bool areaAFree = true, areaBFree = true;
    for (unsigned int i=0;i< obstacles.size();i++) {
      if (obstacles[i].player>=0) continue; // eigene mitspieler ignorieren
      if (towardsGoal.is_inside(obstacles[i].pos) ||
          (towardsGoal.closest_point(obstacles[i].pos)-obstacles[i].pos).length() < obstacles[i].width /2.) {
        areaAFree = false;
      }
      if (forward.is_inside(obstacles[i].pos) ||
          (forward.closest_point(obstacles[i].pos)-obstacles[i].pos).length() < obstacles[i].width / 2.) {
        areaBFree = false;
        break;
      }
    }
    LOUT << "\% " << (areaBFree ? "grey " : "red ") << forward << endl;
    LOUT << "\% " << (areaAFree ? "grey " : "red ") << towardsGoal << endl;
    // deutlich aus dem  Bereich raus und Umgebung frei
    if (((fabs(robotPos.x) < field.field_width/3. - 500. &&
          fabs(robotPos.y) < field.field_length/2. -
                              field.penalty_area_length/2. - 500.) ||
         fabs(robotPos.y) < field.field_length/2. - 
                             field.penalty_area_length*1.5 - 1000.) &&
        areaAFree && areaBFree) { 
      return false;                  // kann aufhoeren, genuegend Platz
    } 
    return true;
  }
}

