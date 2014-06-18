#include "BDoubleTeam.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

#define linedistance 270

namespace Tribots
{
  using namespace std;
    
  BDoubleTeam::BDoubleTeam(const string name) 
    : Behavior(name), skill(new SKILL()), 
      headingController(4., 0.0000 , 0.00, 3., -3.)
  {
    transVel=2.5;
  }

  BDoubleTeam::~BDoubleTeam() throw ()
  {
    delete skill;
  }

  void BDoubleTeam::updateTactics (const TacticsBoard& tb) 
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
        transVel=2.8;
    } else { // =normal
        transVel=2.5;
    }
  } 
  
  DriveVector
  BDoubleTeam::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    
    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec relPos;
    if (ballLocation.pos.x > 0) {
      relPos = Vec(-1100., -300.);
    }
    else {
      relPos = Vec(+1100., -300.);
    }
    if (fabs(ballLocation.pos.x) > 2000.) {        // geht der ball sehr weit auf die Seite, Abstand vergroeﬂern
      relPos *= (fabs(ballLocation.pos.x) - 2000.) / 2000. + 1.;
    }
    Vec target = ballLocation.pos.toVec() + relPos;
    
    // Dieser Spieler darf nicht in den Strafraum rein. //
    XYRectangle oppPenaltyArea(Vec(-field.penalty_area_width/2. - linedistance, 
                                   +field.field_length / 2 + 5000), //bis 5m hinter torlinie
                               Vec(field.penalty_area_width/2. + linedistance,
                                   +field.field_length / 2. - linedistance -
                                   field.penalty_area_length));
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
   
    if (penaltyArea.is_inside(target)) {     // Ball in Strafraum
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
    if (oppPenaltyArea.is_inside(target)) {   // nicht in den gegnerischen strafraum
      target.y = field.field_length/2. - field.penalty_area_length - linedistance;
    }
    
    // check, if position is free, choose position behind and to middle, 
    // if not.
    bool posFree = true;
    for (unsigned int i=0; i < obstacles.size(); i++) {
      if ((obstacles[i].pos-target).length() < 300.) {
        posFree = false;
        break;
      }
    }

	//hindernis ausweichen
	Vec tempLocation;
    if (!posFree) {
      if (ballLocation.pos.x > 0) {
        tempLocation = target + Vec(-200., -200);  // links hinter dem anderen
      }
      else {
        tempLocation = target + Vec(200., -200);   //rechts hinter dem anderen
      }
	  	  //wenn der roboter durch ausweichen in die penalty area fahren würde, dann clippen, damit er draussen bleibt
	  if (penaltyArea.is_inside(tempLocation)) {
		if (target.y > (-field.field_length / 2. + linedistance/2. + field.penalty_area_length)) { 
	  	  tempLocation.y=target.y; //vor der penalty area bleiben
	    }
	  }
	  target = tempLocation;
    }


    skill->init(target, transVel, ballLocation.pos.toVec()-robot.pos);
    return skill->getCmd(t);  
  }

  bool 
  BDoubleTeam::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return seheBall || seheKommuniziertenBall;
  }

  bool BDoubleTeam::checkCommitmentCondition(const Time& tt) throw()
  {
    return BDoubleTeam::checkInvocationCondition(tt);
  }
}

