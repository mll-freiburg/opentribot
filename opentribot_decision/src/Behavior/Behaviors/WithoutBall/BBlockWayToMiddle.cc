#include "BBlockWayToMiddle.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

namespace Tribots
{
  using namespace std;
    
  BBlockWayToMiddle::BBlockWayToMiddle() 
    : Behavior("BBlockWayToMiddle"), skill(new SGoToPosEvadeObstacles())
  {
    transVel=2.5;
    lastTimeBefreiungsschlag.update();
    lastTimeBefreiungsschlag.add_sec(-5);
  }

  BBlockWayToMiddle::~BBlockWayToMiddle() throw ()
  {
    delete skill;
  }
  
  void BBlockWayToMiddle::cycleCallBack(const Time& t) throw()
  {
    if (MWM.get_message_board().scan_for_prefix ("Befreiungsschlag!").length()>0) {
      LOUT << "Befreiungsschlag in BBlockWayToMiddle gehoert." << endl;
      lastTimeBefreiungsschlag.update();
    }
  }
  
  void BBlockWayToMiddle::updateTactics (const TacticsBoard& tb) 
    throw () 
  {
    // AllgemeineFahrgeschwindigkeit = schneller *normal langsamer langsam
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsamer") {
         transVel=2.;
    } else	  
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "langsam") {
        transVel=1.5;
    } else
    if (tb[string("AllgemeineFahrgeschwindigkeit")] == "schneller") {
        transVel=2.8;
    } else { // =normal
        transVel=2.5;
    }
  }
  
  void 
  BBlockWayToMiddle::gainControl(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    blockPosition = ballLocation.pos.x > 0 ? BLOCK_LEFT : BLOCK_RIGHT;
  }      

  DriveVector
  BBlockWayToMiddle::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);
    const ObstacleLocation& obstacles = MWM.get_obstacle_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec blockingPos;
    if (t.diff_sec(lastTimeBefreiungsschlag) < 2) { // Befreiungsschlag gehoert
      blockingPos = (blockPosition == BLOCK_LEFT ? Vec(-2000, 2000) : Vec(2000, 2000));
    }
    else if (t.diff_sec(lastTimeBefreiungsschlag) < 4 && ballLocation.pos.y > 0 &&
          !WBOARD->teamPossessBall() && !WBOARD->teammateNearBall()) {
      blockingPos = (blockPosition == BLOCK_LEFT ? Vec(-2000, 2000) : Vec(2000, 2000));
    }
    else if (WBOARD->teamPossessBall()) {
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
    for (unsigned int i=0; i < obstacles.size(); i++) {
      if ((obstacles[i].pos-blockingPos).length() < 600.) {
        posFree = false;
        break;
      }
    }

    if (!posFree) { 
      blockingPos.x -= 600.; // vertrauen ist gut... TODO: Kontrolle...
    }
    
    // Ausweichposition berechnen, wenn zwischen aktueller Position und Ziel ein Mitspieler mit Ball ist
    
    if ((WBOARD->teamPossessBall() || WBOARD->teammateNearBall()) &&
        (ballLocation.pos_known == BallLocation::known ||
         ballLocation.pos_known == BallLocation::communicated) &&
        ballLocation.pos.y < robot.pos.y) {
      Quadrangle toPos(robot.pos, blockingPos, 2000);
      LOUT << "\% yellow thin solid " << toPos << endl;
      if (toPos.is_inside(ballLocation.pos.toVec())) {
        Frame2d quad2world( robot.pos, (blockingPos - robot.pos).angle() - Angle::quarter);
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
        blockingPos = absAvoidPos;
        LOUT << "\% black solid thick " << Circle(blockingPos, 150) << endl;
      }
    }
    
    skill->init(blockingPos, transVel, ballLocation.pos.toVec()-robot.pos);
    return skill->getCmd(t);
  }

  bool 
  BBlockWayToMiddle::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return seheBall || seheKommuniziertenBall;
  }

  bool BBlockWayToMiddle::checkCommitmentCondition(const Time& tt) throw()
  {
    const BallLocation& ballLocation = MWM.get_ball_location(tt);
    return 
      BBlockWayToMiddle::checkInvocationCondition(tt) && 
      ((blockPosition == BLOCK_LEFT && ballLocation.pos.x > -1000.) ||
       (blockPosition == BLOCK_RIGHT && ballLocation.pos.x < 1000.));
  }
}

