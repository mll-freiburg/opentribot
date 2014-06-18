#include "BSupportDoubleTeamSideline.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

#define MAX(x,y) (x)>(y)?(x):(y)
#define MIN(x,y) (x)<(y)?(x):(y)

#define linedistance 270

namespace Tribots
{
  using namespace std;
    
  BSupportDoubleTeamSideline::BSupportDoubleTeamSideline(string name) 
    : Behavior(name), skill(new SKILL()), 
      headingController(4., 0.0000 , 0.00, 3., -3.)
  {
    transVel=2.5;
  }

  BSupportDoubleTeamSideline::~BSupportDoubleTeamSideline() throw ()
  {
    delete skill;
  }

  void BSupportDoubleTeamSideline::updateTactics (const TacticsBoard& tb) 
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
  BSupportDoubleTeamSideline::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);
    const RobotLocation& robot = MWM.get_robot_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    //  1,5 meter seitlich und 2,8 meter hinter dem ball (auf einem 18m langen feld)
    Vec targetPos = 
      ballLocation.pos.toVec() + Vec(ballLocation.pos.x<0?-field.field_width/12.:field.field_width/12, -field.field_length/6.42); 
    targetPos.x = MIN(MAX(targetPos.x, -field.field_width/2.),
                      field.field_width/2.);
    targetPos.y = MIN(MAX(targetPos.y, -field.field_length/2.),
                      field.field_length/2.);
  
    // An Strafraumgrenze warten....
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
        
    if (penaltyArea.is_inside(targetPos)) {     // Position im Strafraum
      // sucht den Punkt auf Hoehe / Breite des Roboters am Strafraum,
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
        targetPos = possiblePoints[closest];
        if (targetPos.y < -field.field_length/2) {
          targetPos.y = -field.field_length/2;
        }
      }
    }
     
    // check, if position is free, choose position in front and to outside of field, 
    // if not.
    bool posFree = true;
    for (unsigned int i=0; i < obstacles.size(); i++) {
      if ((obstacles[i].pos-targetPos).length() < 500.) {
        posFree = false;
        break;
      }
    }

    if (!posFree) {
      if (robot.pos.x > 0) {             // da bleiben, wo der roboter ist
        targetPos += Vec(700., 200);
      }
      else {
        targetPos += Vec(-700., 200.);
      }
    }
    
    // Ausweichposition berechnen, wenn zwischen aktueller Position und Ziel ein Mitspieler mit Ball ist
    
    if ((WBOARD->teamPossessBall() || WBOARD->teammateNearBall()) &&
        (ballLocation.pos_known == BallLocation::known ||
         ballLocation.pos_known == BallLocation::communicated) &&
        ballLocation.pos.y < robot.pos.y) {
      Quadrangle toPos(robot.pos, targetPos, 2000);
      LOUT << "\% yellow thin solid " << toPos << endl;
      if (toPos.is_inside(ballLocation.pos.toVec())) {
        Frame2d quad2world( robot.pos, (targetPos - robot.pos).angle() - Angle::quarter);
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
        targetPos = absAvoidPos;
        LOUT << "\% black solid thick " << Circle(targetPos, 150) << endl;
      }
    }

    skill->init(targetPos, transVel, ballLocation.pos.toVec()-robot.pos);
    return skill->getCmd(t);  
  }

  bool 
  BSupportDoubleTeamSideline::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return seheBall || seheKommuniziertenBall;
  }

  bool BSupportDoubleTeamSideline::checkCommitmentCondition(const Time& tt) throw()
  {
    return BSupportDoubleTeamSideline::checkInvocationCondition(tt);
  }
}

