#include "BSafety.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

#define linedistance 270

namespace Tribots
{
  using namespace std;
    
  BSafety::BSafety(string name) 
    : Behavior(name), skill(new SKILL()), 
      headingController(4., 0.0000 , 0.00, 3., -3.)
  {
    const FieldGeometry& field = MWM.get_field_geometry(); 
    transVel=2.5;
    protectPos = Vec(0., -500.-field.field_length / 2.);
  }

  BSafety::~BSafety() throw ()
  {
    delete skill;
  }

  void BSafety::updateTactics (const TacticsBoard& tb) 
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
  BSafety::getCmd(const Time& t) throw(TribotsException)
  {
    // Idea: Pong, when ball inside penalty area, stay at penalty area border 
    // in other case and block short corner. (like a second goalie)
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
    
    // Ausname: Ball im Strafraum -> Pong 50cm 
    Vec pongLocation=ballLocation.pos.toVec()+
      (protectPos-ballLocation.pos.toVec()).normalize()*500.;//50cm vom Ball weg
  
    // Der ball-spieler darf in die area rein, waehrend die beiden 
    // pong spieler draussen bleiben muessen.
    XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - linedistance, 
                                -field.field_length / 2. - 5000), // bis 5m hinter linie
                            Vec(field.penalty_area_width/2. + linedistance,
                                -field.field_length / 2. + linedistance +
                                field.penalty_area_length));
    
    LineSegment leftLineOfPenaltyArea(Vec(-field.penalty_area_width/2. - linedistance, 
                                          -field.field_length / 2. - 5000),
                                      Vec(-field.penalty_area_width/2. - linedistance, 
                                          -field.field_length / 2. + 
                                          field.penalty_area_length + linedistance));
    LineSegment rightLineOfPenaltyArea(Vec(field.penalty_area_width/2. + linedistance, 
                                           -field.field_length / 2. - 5000),
                                       Vec(field.penalty_area_width/2. + linedistance, 
                                           -field.field_length / 2. + 
                                           field.penalty_area_length + linedistance));
    LineSegment topLineOfPenaltyArea(Vec(-field.penalty_area_width/2. - linedistance, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length + linedistance),
                                     Vec(field.penalty_area_width/2.+ linedistance, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length + linedistance));
    
    // Regelfall: Ball nicht in Strafraum
    if (!penaltyArea.is_inside(ballLocation.pos.toVec())) {
      // sucht den Punkt an der Strafraumgrenze, der auf der abzudeckenden Linie
      // Ball->Tor liegt.
      LineSegment ballLine(ballLocation.pos.toVec(), protectPos);
      
      vector<Vec> intersections;
      intersections = intersect(leftLineOfPenaltyArea, ballLine);
      if (! intersections.size()) {
        intersections = intersect(rightLineOfPenaltyArea, ballLine);
        if (! intersections.size()) {
          intersections = intersect(topLineOfPenaltyArea, ballLine);
        }
      }
      if (intersections.size() > 0) {
        pongLocation = intersections[0];
      } else {
        cerr << "No intersection found" << endl;
      }
    }
        
    // kein check, ob Position frei! Anderen Roboter sollen ausweichen
    // keine Ausweichposition berechnen!
    
    skill->init(pongLocation, transVel, ballLocation.pos.toVec()-robot.pos);
    return skill->getCmd(t);  
  }

  bool 
  BSafety::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return seheBall || seheKommuniziertenBall;
  }

  bool BSafety::checkCommitmentCondition(const Time& tt) throw()
  {
    return BSafety::checkInvocationCondition(tt);
  }
}

