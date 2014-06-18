#include "BBlockWayToGoal.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

namespace Tribots
{
  using namespace std;
    
  BBlockWayToGoal::BBlockWayToGoal(double maxDistance, bool obeyPenaltyAreaRules) 
    : Behavior("BBlockWayToGoal"), skill(new SKILL()), 
      headingController(4., 0.0000 , 0.00, 3., -3.), maxDistance(maxDistance),
      obeyPenaltyAreaRules(obeyPenaltyAreaRules)
  {
    transVel=2.5;
  }

  BBlockWayToGoal::~BBlockWayToGoal() throw ()
  {
    delete skill;
  }

  void BBlockWayToGoal::updateTactics (const TacticsBoard& tb) 
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
  BBlockWayToGoal::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec ownGoalPos(0., -300-field.field_length / 2.);
    Vec pongLocation=ownGoalPos+(ballLocation.pos.toVec()-ownGoalPos)*0.75;
  
    // Der ball-spieler darf in die area rein, waehrend die beiden 
    // pong spieler draussen bleiben muessen.
    XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - 200, 
                                -field.field_length / 2. - 3000), // hinter gl
                            Vec(field.penalty_area_width/2. + 200,
                                -field.field_length / 2. + 200 +
                                field.penalty_area_length));
    
    LineSegment leftLineOfPenaltyArea(Vec(-field.penalty_area_width/2.-200, 
                                          -field.field_length / 2. - 200.),
                                      Vec(-field.penalty_area_width/2.-200, 
                                          -field.field_length / 2. + 
                                          field.penalty_area_length+100));
    LineSegment rightLineOfPenaltyArea(Vec(field.penalty_area_width/2.+200, 
                                           -field.field_length / 2. - 200.),
                                       Vec(field.penalty_area_width/2.+200, 
                                           -field.field_length / 2. + 
                                           field.penalty_area_length+100));
    LineSegment topLineOfPenaltyArea(Vec(-field.penalty_area_width/2.-200, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length+100),
                                     Vec(field.penalty_area_width/2.+200, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length+100));
    
    if (penaltyArea.is_inside(ballLocation.pos.toVec()) &&  // TODO: ERROR: Ball ist hier egal
        penaltyArea.is_inside(pongLocation) && obeyPenaltyAreaRules) {    // Ball in Strafraum
      // sucht den Punkt auf Hoehe / Breite des Balles am Strafraum,
      // der am naechsten dran an der aktuellen Roboterposition ist.
      // Idee ist, der Roboter kommt schon aus einer sinnvollen Richtung...
      LineSegment segments[3] = {
        leftLineOfPenaltyArea, rightLineOfPenaltyArea, topLineOfPenaltyArea
      };
      Vec dirs[3] = { Vec(1., 0), Vec(1., 0), Vec(0., 1) };
      vector<Vec> possiblePoints;
      for (unsigned int i=0; i < 3; i++) {
        vector<Vec> intersections = intersect(segments[i],  // TODO: ERROR: Wenn der Ball noch draussen ist, besser auf Linie bleiben!
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
        pongLocation = possiblePoints[closest];
        if (pongLocation.y < -field.field_length/2) {
          pongLocation.y = -field.field_length/2;
        }
      }
    }
    else if (penaltyArea.is_inside(pongLocation)) {// Roboter soll in Strafraum
      // sucht den Punkt an der Strafraumgrenze, der auf der abzudeckenden Linie
      // Ball->Tor liegt.
      LineSegment ballLine(ballLocation.pos.toVec(), ownGoalPos);
      
      vector<Vec> intersections;
      intersections = intersect(leftLineOfPenaltyArea, ballLine);
      if (! intersections.size()) {
        intersections = intersect(rightLineOfPenaltyArea, ballLine);
      }
      if (! intersections.size()) {
        intersections = intersect(topLineOfPenaltyArea, ballLine);
      }
      if (intersections.size() >= 1) {
        pongLocation = intersections[0];
      }
    }
    
    if (((pongLocation-ownGoalPos).length()) > maxDistance) {
      pongLocation = ownGoalPos + 
        (ballLocation.pos.toVec()-ownGoalPos).normalize() * maxDistance; 
    }
    
    // check, if position is free, choose position behind and to middle, 
    // if not.
    bool posFree = true;
    for (unsigned int i=0; i < obstacles.size(); i++) {
      if ((obstacles[i].pos-pongLocation).length() < 500.) {
        posFree = false;
        break;
      }
    }

    if (!posFree) {
      if (ballLocation.pos.x > 200.) {
        pongLocation += Vec(-400., -700);  // links hinter dem anderen
      }
      else if (ballLocation.pos.x < -200.) {
        pongLocation += Vec(400., -700);   //rechts hinter dem anderen
      }
      else { 
        if (robot.pos.x > 0) {             // da bleiben, wo der roboter ist
          pongLocation += Vec(400., -700.);
        }
        else {
          pongLocation += Vec(-400., -700.);
        }
      }
    }
    
    // Ausweichposition berechnen, wenn zwischen aktueller Position und Ziel ein Mitspieler mit Ball ist
    
    if ((WBOARD->teamPossessBall() || WBOARD->teammateNearBall()) &&
        (ballLocation.pos_known == BallLocation::known ||
         ballLocation.pos_known == BallLocation::communicated) &&
        ballLocation.pos.y < robot.pos.y) {
      Quadrangle toPos(robot.pos, pongLocation, 2000);
      LOUT << "\% yellow thin solid " << toPos << endl;
      if (toPos.is_inside(ballLocation.pos.toVec())) {
        Frame2d quad2world( robot.pos, (pongLocation - robot.pos).angle() - Angle::quarter);
        Frame2d world2quad= quad2world;
        world2quad.invert();
        
        Vec relBallPos = world2quad * ballLocation.pos.toVec();
        Vec relAvoidPos; 
        if (relBallPos.x > 0) {
          relAvoidPos = relBallPos - Vec(2000.,0);
        }
        else {
          relAvoidPos = relBallPos + Vec(2000.,0);
        }
        Vec absAvoidPos = quad2world * relAvoidPos;
        if (fabs(absAvoidPos.x) > field.field_width/2 + 300) {
          absAvoidPos.x = (field.field_width/2. + 300) * absAvoidPos.x/fabs(absAvoidPos.x);
        }
        pongLocation = absAvoidPos;
        LOUT << "\% black solid thick " << Circle(pongLocation, 150) << endl;
      }
    }

    skill->init(pongLocation, transVel, ballLocation.pos.toVec()-ownGoalPos);
    return skill->getCmd(t);  
  }

  bool 
  BBlockWayToGoal::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return seheBall || seheKommuniziertenBall;
  }

  bool BBlockWayToGoal::checkCommitmentCondition(const Time& tt) throw()
  {
    return BBlockWayToGoal::checkInvocationCondition(tt);
  }
}

