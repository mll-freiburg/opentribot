#include "BSupportDoubleTeamMiddle.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

#define linedistance 270

namespace Tribots
{
  using namespace std;
    
  BSupportDoubleTeamMiddle::BSupportDoubleTeamMiddle(bool obeyPenaltyAreaRules, string name) 
    : Behavior(name), skill(new SKILL()), 
      headingController(4., 0.0000 , 0.00, 3., -3.),
      obeyPenaltyAreaRules(obeyPenaltyAreaRules)
  {
    const FieldGeometry& field = MWM.get_field_geometry();
    protectPos = Vec(0, -field.field_length/2.-300.);
    transVel=2.5;
    minDistanceToBall = 2500.;
    maxDistanceToBall = 100000.; // zur zeit kein maximaler abstand
  }

  BSupportDoubleTeamMiddle::~BSupportDoubleTeamMiddle() throw ()
  {
    delete skill;
  }

  void BSupportDoubleTeamMiddle::updateTactics (const TacticsBoard& tb) 
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
        transVel=3.0;
    } else { // =normal
        transVel=2.5;
    }
  } 
  
  DriveVector
  BSupportDoubleTeamMiddle::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec toTargetPos = (protectPos-ballLocation.pos.toVec()) * .36;
    if (toTargetPos.length() < minDistanceToBall) {
      toTargetPos = toTargetPos.normalize() * minDistanceToBall;
    }
    Vec pongLocation=ballLocation.pos.toVec()+toTargetPos;
  
  	Vec orientation = ballLocation.pos.toVec()-protectPos;

    // Der ball-spieler darf in die area rein, waehrend die beiden 
    // pong spieler draussen bleiben muessen.
    XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - linedistance, 
                                -field.field_length / 2 - 5000), //bis 5m hinter torlinie
                            Vec(field.penalty_area_width/2. + linedistance,
                                -field.field_length / 2. + linedistance +
                                field.penalty_area_length));

 	XYRectangle behindField(Vec(-field.field_width/2. - 5000,  //gesamten bereich hinterm tor abdecken
                                -field.field_length / 2. - 5000),
                            Vec(field.field_width/2. + 5000,
                                -field.field_length / 2.-500));	

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
	LineSegment baseLine(Vec(-field.field_width/2. - 5000, 
							 -field.field_length / 2.),
						 Vec(field.field_width/2. + 5000, 
							 -field.field_length / 2.));

	if (behindField.is_inside(ballLocation.pos.toVec()) 
	    && !(penaltyArea.is_inside(ballLocation.pos.toVec())) ) {

		//ball auf grundlinie projezieren
		vector<Vec> intersections = intersect(baseLine,
                                              Line(ballLocation.pos.toVec(), ballLocation.pos.toVec() + Vec(0, 1.)));

		if (intersections.size()) {
			if (intersections[0].x < -field.field_width/2) {
				intersections[0].x = -field.field_width/2;
			}
			if (intersections[0].x > field.field_width/2) {
				intersections[0].x = field.field_width/2;
			}

			pongLocation = intersections[0];
			orientation = ballLocation.pos.toVec()-robot.pos;
			
		} else {
			LOUT << "Schnittpunkt mit Grundlinie kann nicht berechnet werden" << endl;
		}

	} else {
      if (penaltyArea.is_inside(ballLocation.pos.toVec()) &&
          penaltyArea.is_inside(pongLocation) && obeyPenaltyAreaRules) {     // Position im Strafraum
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
          pongLocation = possiblePoints[closest];
          if (pongLocation.y < -field.field_length/2) {
            pongLocation.y = -field.field_length/2;
          }
        }

	    orientation = ballLocation.pos.toVec()-robot.pos;

      }
      else if (penaltyArea.is_inside(pongLocation)) {// Roboter soll in Strafraum
        // sucht den Punkt an der Strafraumgrenze, der auf der abzudeckenden Linie
        // Ball->Tor liegt.
        LineSegment ballLine(ballLocation.pos.toVec(), pongLocation);
      
        //cerr << "Pong in Penalty area: " << ballLine << ", " << pongLocation << ", " << ballLocation.pos.toVec() << endl;
      
        vector<Vec> intersections;
        intersections = intersect(topLineOfPenaltyArea, ballLine);
        if (! intersections.size()) {
          intersections = intersect(rightLineOfPenaltyArea, ballLine);
          if (! intersections.size()) {
            intersections = intersect(leftLineOfPenaltyArea, ballLine);
          }
        }
        if (intersections.size() > 0) {
          pongLocation = intersections[0];
        } else {
          cerr << "No intersection found" << endl;
        }
      }
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

	//hindernis ausweichen
	Vec tempLocation;
    if (!posFree) {
      if (ballLocation.pos.x > 200.) {
        tempLocation = pongLocation + Vec(-400., -700);  // links hinter dem anderen
//		cout << "muss nach hinten links ausweichen" << endl;
      }
      else if (ballLocation.pos.x < -200.) {
        tempLocation = pongLocation + Vec(400., -700);   //rechts hinter dem anderen
//		cout << "muss nach hinten rechts ausweichen" << endl;
      }
      else { 
        if (robot.pos.x > 0) {             // da bleiben, wo der roboter ist
          tempLocation = pongLocation + Vec(400., -700.);
//		cout << "muss nach hinten links ausweichen (2)" << endl;
        }
        else {
          tempLocation = pongLocation + Vec(-400., -700.);
//		cout << "muss nach hinten rechts ausweichen (2)" << endl;
        }
      }
	  //wenn der roboter durch ausweichen in die penalty area fahren w�rde, dann clippen, damit er draussen bleibt
	  if (penaltyArea.is_inside(tempLocation)) {
		if (pongLocation.y > (-field.field_length / 2. + linedistance/2. + field.penalty_area_length)) { //will von vorne in die PA hinein einem gegner ausweichen
	  	  tempLocation.y=pongLocation.y; //vor der penalty area bleiben
	    }
	  	if (pongLocation.x < (-field.penalty_area_width/2.- linedistance/2.)) { //will von der linken seite der PA aus ausweichen
	  	  tempLocation.x=pongLocation.x;
	    } else if (pongLocation.y > (field.penalty_area_width/2.+ linedistance/2.)) {
	  	  tempLocation.x=pongLocation.x;
	    }
	  }
	  pongLocation = tempLocation;
    }
    
    // Ausweichposition berechnen, wenn zwischen aktueller Position und Ziel ein Mitspieler mit Ball ist
    
    if ((WBOARD->teamPossessBall()) &&
        (ballLocation.pos_known == BallLocation::known ||
         ballLocation.pos_known == BallLocation::communicated) &&
        ballLocation.pos.y < robot.pos.y) {
      Quadrangle toPos(robot.pos, pongLocation, 1000);
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

	if ((pongLocation - robot.pos).length() < 100) {
		skill->init(robot.pos, transVel, orientation);
	} else {
	    skill->init(pongLocation, transVel, orientation);
	}
    return skill->getCmd(t);  
  }

  bool 
  BSupportDoubleTeamMiddle::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return seheBall || seheKommuniziertenBall;
  }

  bool BSupportDoubleTeamMiddle::checkCommitmentCondition(const Time& tt) throw()
  {
    return BSupportDoubleTeamMiddle::checkInvocationCondition(tt);
  }
}

