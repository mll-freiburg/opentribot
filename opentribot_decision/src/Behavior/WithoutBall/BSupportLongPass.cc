#include "BSupportLongPass.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include <iostream>
#include <cmath>

namespace Tribots
{
  using namespace std;
    
  BSupportLongPass::BSupportLongPass(const string name) 
    : Behavior(name), skill(new SGoToPosEvadeObstacles()),
    fast_skill(new SBoostToPos(1500, false, 0.3)), // tatsaechlich wird der erste wert (abstand) derzeit ignoriert und der winkel wird nur beachtet, wenn man einen ball dabei hat
    activeSkill(0), presentTactics(TACTICS_OFFENSIVE), boostAllowed(true)
  {
    transVel=2.5;
  }

  BSupportLongPass::~BSupportLongPass() throw ()
  {
    delete skill;
    delete fast_skill;
  }
  
  void BSupportLongPass::updateTactics (const TacticsBoard& tb) 
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
    
    if (tb[string("boosting")] == string("nein")) {
      boostAllowed = false;
    }
    else {
      boostAllowed = true;
    }
    
    if (tb[string("TaktischeAusrichtung")] == string("defensiv")) {
      presentTactics = TACTICS_DEFENSIVE;
    }
    else if (tb[string("TaktischeAusrichtung")] == string("neutral")) {
      presentTactics = TACTICS_NEUTRAL;
    }
    else {
      presentTactics = TACTICS_OFFENSIVE;
    }
  }
  
  void 
  BSupportLongPass::gainControl(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    blockPosition = ballLocation.pos.x > 0 ? BLOCK_LEFT : BLOCK_RIGHT;

    activeSkill = 0;
  }      

  DriveVector
  BSupportLongPass::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec blockingPos;
/*
    if (ballLocation.pos.y < 0) {  // eigen haelfte
      blockingPos = ballLocation.pos.toVec() +
      (blockPosition == BLOCK_LEFT ? Vec(-3000, field.field_length/18.) : Vec(3000, field.field_length/18.));
    }
    else if (fabs(ballLocation.pos.x) > field.field_width/2. * .6) { // ball nahe seitenlinie
      blockingPos = Vec((blockPosition==BLOCK_LEFT?-.6:.6)* field.field_width/2.,
                        ballLocation.pos.y+field.field_length/12.);
    }
    else {                             // sonst
      blockingPos = Vec((blockPosition==BLOCK_LEFT?-.8:.8)* field.field_width/2.,
                        ballLocation.pos.y-field.field_length/12.);
    }
    
    // Im Feld bleiben
    if (blockingPos.y < -field.field_length/2. + 1600.) { // nicht zu nah
      blockingPos.y = -field.field_length/2. + 1600.;   // zur grundlinie, 
    }                                                 // nicht in strafraum
    else if (blockingPos.y > field.field_length/2. - field.penalty_area_length-500.) {
      blockingPos.y = field.field_length/2. - field.penalty_area_length-500.;
    }
    if (fabs(blockingPos.x) > field.field_width/2.) {
      blockingPos.x = (blockPosition==BLOCK_LEFT?-1:1) * field.field_width/2.;
    }
*/

    blockingPos = Vec((field.penalty_area_width/2.0 + field.field_width/12.0) * (blockPosition == BLOCK_LEFT ? -1 : 1), (field.field_length/2.0) - field.penalty_area_length);
        
    // check, if position is free, choose position behind and to middle, 
    // if not.
    bool posFree = true;
    const ObstacleLocation obstacles = MWM.get_obstacle_location(t);
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
    LOUT << "\% blue solid thick " << Circle(blockingPos, 200) << endl; 

    Skill* newSkill = skill;

    fast_skill->setTargetPos(blockingPos);
    skill->init(blockingPos, transVel, ballLocation.pos.toVec()-robot.pos);

    bool ballInRange = (fabs(ballLocation.pos.x) > field.penalty_area_width/2.0) && (fabs(ballLocation.pos.x) < field.field_width/2.0);
    //wenn ball seitlich der penalty area und roboter nicht zu schnell
    //dann fast skill
    if (boostAllowed && ballInRange && robot.vtrans.length() > .7 &&
        fast_skill->checkInvocationCondition(t)) { //  && drivingLaneFree) {
        newSkill = fast_skill;
    }
    
    if (activeSkill != newSkill) {
      if (activeSkill) {
        activeSkill->loseControl(t);
      }
      newSkill->gainControl(t);
      activeSkill = newSkill;
    }
    return activeSkill->getCmd(t);

  }

  bool 
  BSupportLongPass::checkInvocationCondition(const Time& tt) throw()
  {
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return presentTactics == TACTICS_OFFENSIVE && (seheBall || seheKommuniziertenBall);
  }

  bool BSupportLongPass::checkCommitmentCondition(const Time& tt) throw()
  {
    const BallLocation& ballLocation = MWM.get_ball_location(tt);
    return 
      BSupportLongPass::checkInvocationCondition(tt) && 
      ((blockPosition == BLOCK_LEFT && ballLocation.pos.x > -1000.) ||
       (blockPosition == BLOCK_RIGHT && ballLocation.pos.x < 1000.));
  }
}

