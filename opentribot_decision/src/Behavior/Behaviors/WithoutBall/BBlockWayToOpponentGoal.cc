#include "BBlockWayToOpponentGoal.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

namespace Tribots
{
  using namespace std;
    
  BBlockWayToOpponentGoal::BBlockWayToOpponentGoal() 
    : Behavior("BBlockWayToOpponentGoal"), skill(new SGoToPosEvadeObstacles())
  {
    transVel=2.5;
  }

  BBlockWayToOpponentGoal::~BBlockWayToOpponentGoal() throw ()
  {
    delete skill;
  }

  void BBlockWayToOpponentGoal::updateTactics (const TacticsBoard& tb) 
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
  
  void 
  BBlockWayToOpponentGoal::gainControl(const Time& t) throw(TribotsException)
  {
/*
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    blockPosition = ballLocation.pos.x > 0 ? BLOCK_LEFT : BLOCK_RIGHT;
*/
  }      

  DriveVector
  BBlockWayToOpponentGoal::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec blockingPos;

    if (WBOARD->teamPossessBall()) 
    {
      blockingPos = Vec(0.0, field.field_length) - ballLocation.pos.toVec();
      blockingPos.normalize() * 1000.0;
    }
/*
      if (ballLocation.pos.y < -1000) {  // eigen haelfte
        blockingPos = ballLocation.pos.toVec() +
          (blockPosition == BLOCK_LEFT ? Vec(-3000, -500) : Vec(3000, -500));
      }
      else if (fabs(ballLocation.pos.x) > field.field_width/2. * .6) { // ball nahe seitenlinie
        blockingPos = Vec((blockPosition==BLOCK_LEFT?-.3:.3)* field.field_width/2.,
                          ballLocation.pos.y);
      }
      else {                             // sonst
        blockingPos = Vec((blockPosition==BLOCK_LEFT?-.6:.6)* field.field_width/2.,
                          ballLocation.pos.y);
      }
    }
    else {
      blockingPos = ballLocation.pos.toVec() +
      (blockPosition == BLOCK_LEFT ? Vec(-1800, -1000.) : Vec(1800, -1000.));
    }
    
    
    // Im Feld bleiben
    if (blockingPos.y < -field.field_length/2. + 1600.) { // nicht zu nah
      blockingPos.y = -field.field_length/2. + 1600.;   // zur grundlinie, 
    }                                                 // nicht in strafraum
    else if (blockingPos.y > field.field_length/2. - 1600.) {
      blockingPos.y = field.field_length/2. - 1600.;
    }
    if (fabs(blockingPos.x) > field.field_width/2.) {
      blockingPos.x = (blockPosition==BLOCK_LEFT?-1:1) * field.field_width/2.;
    }

    
    // check, if position is free, choose position behind and to middle, 
    // if not.
    bool posFree = true;
    const ObstacleLocation obstacles = MWM.get_obstacle_location(t);
    for (unsigned int i=0; i < obstacles.pos.size(); i++) {
      if ((obstacles.pos[i]-blockingPos).length() < 600.) {
        posFree = false;
        break;
      }
    }

    if (!posFree) { 
      blockingPos.x -= 600.; // vertrauen ist gut... TODO: Kontrolle...
    }
*/
    skill->init(blockingPos, transVel, ballLocation.pos.toVec()-robot.pos);
    return skill->getCmd(t);
  }

  bool 
  BBlockWayToOpponentGoal::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return seheBall || seheKommuniziertenBall;
  }

  bool BBlockWayToOpponentGoal::checkCommitmentCondition(const Time& tt) throw()
  {
//    const BallLocation& ballLocation = MWM.get_ball_location(tt);
    return 
      BBlockWayToOpponentGoal::checkInvocationCondition(tt);
       /*
       && ((blockPosition == BLOCK_LEFT && ballLocation.pos.x > -1000.) ||
       (blockPosition == BLOCK_RIGHT && ballLocation.pos.x < 1000.));
       */
  }
}

