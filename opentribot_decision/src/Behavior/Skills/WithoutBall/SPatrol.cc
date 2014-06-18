#include "SPatrol.h"
#include "../../../Player/WhiteBoard.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Fundamental/geometry.h"
#include "../../../Structures/Journal.h"
#include <vector>


namespace Tribots {
  
  using namespace std;

  SPatrol::SPatrol()
    : Skill("SPatrol"),
      goToPos(new SGoToPosEvadeObstacles()),
      aktion(1),
      pos1(Vec(0.,0.)), pos2(Vec(0.,0.)),
      positions(std::vector<Vec>()),
      forward(true),
      positionIndex(0),
      patrolSpeed(1.4),
      patrolSpeedToFirstPosition(2.4)
  {}

  SPatrol::~SPatrol() throw ()
  {
    delete goToPos;
  }


  void
  SPatrol::setPatrolPositions(const Vec& pos1, const Vec& pos2) 
  {
    this->pos1 = pos1;
    this->pos2 = pos2;
  }


  void SPatrol::setPatrolPositions(const std::vector<Vec>& positions) {
      this->positions = positions;
  }


  void SPatrol::gainControl(const Time& t) throw(TribotsException){
    firstPosition = false;
  }
  
  DriveVector 
  SPatrol::getCmd(const Time& t) throw(TribotsException)
  {
      const RobotLocation& robot = MWM.get_robot_location(t);
      Vec ziel,heading;
      // allow old code, ie. if patrol positions are explicitly given
      if ((pos1 != Vec(0,0)) || (pos2 != Vec(0,0))) {
         const FieldGeometry& field = MWM.get_field_geometry();
         const BallLocation& ball3d = MWM.get_ball_location(t);
         Vec ball(ball3d.pos.x, ball3d.pos.y);
         
         // Orte
         XYRectangle ziel1( pos1-Vec(300,300),pos1+Vec(300,300) );
         XYRectangle ziel2( pos2-Vec(300,300),pos2+Vec(300,300) );
         
         // mitte - rechts - mitte - links - mitte .....
         if (aktion==1 && ziel1.is_inside(robot.pos)) aktion=2;
         if (aktion==2 && ziel2.is_inside(robot.pos)) aktion=1;
         // heading=Vec((field.goal_width/3),(field.field_length / 2));
         
         if (aktion==1)
         {
           ziel=pos1;
           heading=Vec((-field.goal_width/2),(field.field_length / 2));
         } else
         {
           ziel=pos2;
           heading=Vec((field.goal_width/2),(field.field_length / 2));
         }    
         
         goToPos->init(ziel, patrolSpeed, heading,true);
         return goToPos->getCmd(t);
       } else { // new code with many positions

          if (this->positions.size() < 2 )
              throw new TribotsException("SPatrol::getCmd: Die Liste der Positionen ist zu klein (mindestens 2 sind nötig).");
          
          // idea: move along the path and return in inverse direction
          if (this->forward){ // forward direction
              if ( (int)this->positions.size() == this->positionIndex )  { 
                  this->positionIndex--; 
                  this->forward = false; /* change direction*/ 
              } else { // no change of direction necessary
                XYRectangle ziel( this->positions.at(this->positionIndex)-Vec(300,300),
                                  this->positions.at(this->positionIndex)+Vec(300,300) );
                if (ziel.is_inside(robot.pos)) { // current target reached, go on to the next target
                    if(!firstPosition)
                      firstPosition = true;
                    this->positionIndex++;
                } else {
                    // target not yet reached
                    heading = positions.at(positionIndex) - robot.pos;
                    this->goToPos->init(this->positions.at(this->positionIndex),
                    (firstPosition?this->patrolSpeed:this->patrolSpeedToFirstPosition), heading,true);
                    return this->goToPos->getCmd(t);
                }
              }
           } // forward direction handled

          if (!forward){ // backward direction
              if ( this->positionIndex < 0  ) {
                  this->positionIndex=0;  
                  this->forward = true; /* change direction*/ 
              }
              else { // no change of direction necessary
                XYRectangle ziel( this->positions.at(this->positionIndex)-Vec(300,300),
                                  this->positions.at(this->positionIndex)+Vec(300,300) );
                if (ziel.is_inside(robot.pos)) { // current target reached, go on to the next target
                    this->positionIndex--;
                } else {
                    // target not yet reached
                    heading = this->positions.at(this->positionIndex) - robot.pos;
                    this->goToPos->init(this->positions.at(this->positionIndex), this->patrolSpeed, heading,true);
                    return this->goToPos->getCmd(t);
                }
              }
           } // backward direction handled

          // if no action since here:
          return DriveVector(Vec(0,0)); // wait an iteration
       }
  }


}
