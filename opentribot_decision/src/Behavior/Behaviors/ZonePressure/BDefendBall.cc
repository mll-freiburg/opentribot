#include "BDefendBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

#define linedistance 270

namespace Tribots
{
  using namespace std;
    
  BDefendBall::BDefendBall(const string name) 
    : Behavior(name), skill(new SKILL()), 
      headingController(4., 0.0000 , 0.00, 3., -3.)
  {
    transVel=2.5;
  }

  BDefendBall::~BDefendBall() throw ()
  {
    delete skill;
  }

  void BDefendBall::updateTactics (const TacticsBoard& tb) 
    throw () 
  {
    // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
         transVel=1.4;
    } else	  
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
        transVel=1.0;
    } else
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
        transVel=2.5;
    } else { // =normal
        transVel=2.5;
    }
  } 
  
  DriveVector
  BDefendBall::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec target = ballLocation.pos.toVec() + 
      (Vec(0,-field.field_length/2.)-ballLocation.pos.toVec()).normalize() * 500.;
    
    // Dieser Spieler darf nicht in den Strafraum rein, wenn es einen safety gibt. //
    XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - linedistance, 
                                -field.field_length / 2 - 5000), //bis 5m hinter torlinie
                            Vec(field.penalty_area_width/2. + linedistance,
                                -field.field_length / 2. + linedistance +
                                field.penalty_area_length));
    
    LineSegment leftLineOfPenaltyArea(Vec(-field.penalty_area_width/2.- linedistance, 
                                          -field.field_length / 2. - 5000),
                                      Vec(-field.penalty_area_width/2.-linedistance, 
                                          -field.field_length / 2. + 
                                          field.penalty_area_length+linedistance));
    LineSegment rightLineOfPenaltyArea(Vec(field.penalty_area_width/2.+linedistance, 
                                           -field.field_length / 2. - 5000),
                                       Vec(field.penalty_area_width/2.+linedistance, 
                                           -field.field_length / 2. + 
                                           field.penalty_area_length+linedistance));
    LineSegment topLineOfPenaltyArea(Vec(-field.penalty_area_width/2.-linedistance, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length+linedistance),
                                     Vec(field.penalty_area_width/2.+linedistance, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length+linedistance));
    
    if (!WBOARD->onlyOneRobot() && !WBOARD->onlyTwoRobots() &&
        !WBOARD->onlyThreeRobots() && !WBOARD->onlyFourRobots() &&
        penaltyArea.is_inside(target)) {     // Ball in Strafraum
      // sucht den Punkt auf Hoehe / Breite des Balles am Strafraum,
      // der am naechsten dran an der aktuellen Roboterposition ist.
      // Idee ist, der Roboter kommt schon aus einer sinnvollen Richtung...
      LineSegment segments[3] = {
        leftLineOfPenaltyArea, rightLineOfPenaltyArea, topLineOfPenaltyArea
      };
      Vec dirs[3] = { Vec(1., 0), Vec(1., 0), Vec(0., 1) };
      vector<Vec> possiblePoints;
      for (unsigned int i=0; i < 3; i++) {
        vector<Vec> intersections = intersect(segments[i],
                                              Line(robot.pos, robot.pos + dirs[i]));
        if (intersections.size()) {
          possiblePoints.push_back(intersections[0]);
        }
      }
      if (possiblePoints.size() > 0) {
        int closest = 0;
        for (unsigned int i=1; i < possiblePoints.size(); i++) {
          if ((possiblePoints[i]-robot.pos).length() < 
              (possiblePoints[closest]-robot.pos).length())
            closest = i;
        }
        target = possiblePoints[closest];
        if (target.y < -field.field_length/2) {
          target.y = -field.field_length/2;
        }
      }
    }

    skill->init(target, transVel, ballLocation.pos.toVec()-robot.pos);
    return skill->getCmd(t);  
  }

  bool 
  BDefendBall::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return seheBall || seheKommuniziertenBall;
  }

  bool BDefendBall::checkCommitmentCondition(const Time& tt) throw()
  {
    return BDefendBall::checkInvocationCondition(tt);
  }
}

