#include "BSupportNearBall.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../Player/WhiteBoard.h"
#include "../../Predicates/freeCorridor.h"
#include <iostream>
#include <cmath>

namespace Tribots
{
  using namespace std;
    
  BSupportNearBall::BSupportNearBall(const string name) 
    : Behavior(name), skill(new SGoToPosEvadeObstacles()),
    fast_skill(new SBoostToPos(1500, false, 0.3)), // tatsaechlich wird der erste wert (abstand) derzeit ignoriert und der winkel wird nur beachtet, wenn man einen ball dabei hat
    activeSkill(0), presentTactics(TACTICS_OFFENSIVE), boostAllowed(true)
  {
    transVel=2.5;
  }

  BSupportNearBall::~BSupportNearBall() throw ()
  {
    delete skill;
    delete fast_skill;
  }
  
  void BSupportNearBall::updateTactics (const TacticsBoard& tb) 
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
  BSupportNearBall::gainControl(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 

    blockPosition = ballLocation.pos.x > 0 ? BLOCK_LEFT : BLOCK_RIGHT;
    if (ballLocation.pos.x > field.penalty_area_width/2.0) {
      ballInLane = BALL_RIGHT;
    } else if (ballLocation.pos.x < -field.penalty_area_width/2.0) {
      ballInLane = BALL_LEFT;
    } else {
      ballInLane = BALL_MIDDLE;
    }
    
    activeSkill = 0;
  }      

  void 
  BSupportNearBall::loseControl(const Time& t) throw(TribotsException)
  {
    if (activeSkill) activeSkill->loseControl(t);
  }      

  DriveVector
  BSupportNearBall::getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const RobotLocation& robot = MWM.get_robot_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);
    
    if (presentTactics == TACTICS_OFFENSIVE) {
      if (ballLocation.pos.x > field.penalty_area_width/2.0) {
        ballInLane = BALL_RIGHT;
      } else if (ballLocation.pos.x < -field.penalty_area_width/2.0) {
        ballInLane = BALL_LEFT;
      } else if (ballLocation.pos.x > -field.goal_area_width/2.0 && ballLocation.pos.x < field.goal_area_width/2.0) {
        ballInLane = BALL_MIDDLE; //hysteresis
      }
    }
    else {   // nur in TACTICS_OFFENSIVE auf die festen Passpositionen fahren.
      ballInLane = BALL_MIDDLE;
    }
    
    // schauen, ob die blockposition gewechselt werden muss, weil der ballfuehrende spieler zu weit auf die
    // seite dieses Spielers gekommen ist
    if (ballLocation.pos.x < -field.penalty_area_width/2.-500.) { blockPosition = BLOCK_RIGHT; }
    else if (ballLocation.pos.x > field.penalty_area_width/2.+500.) { blockPosition = BLOCK_LEFT; }

    Vec blockingPos;
    if (ballInLane == BALL_LEFT || ballInLane == BALL_RIGHT) {
      blockingPos = Vec(field.goal_width/2.0 * (ballInLane==BALL_LEFT ? -1 : 1), (field.field_length/2.0 - field.penalty_area_length) /2.0);
    
    } else {
      blockingPos = ballLocation.pos.toVec() +
        (blockPosition == BLOCK_LEFT ? Vec(-3000, -1750.) : Vec(3000, -1750.));
    }

    // Im Feld bleiben
    if (blockingPos.y < -field.field_length/2. + field.penalty_area_length + 200) { // nicht zu nah
      blockingPos.y = -field.field_length/2. + field.penalty_area_length + 200;   // zur grundlinie, 
    }                                                 // nicht in strafraum
    if (blockingPos.y > +field.field_length/2. - field.penalty_area_length - 200) { // nicht in
      blockingPos.y = +field.field_length/2. - field.penalty_area_length -200;   // gegn. strafraum
    }                                                 // nicht in strafraum
    else if (blockingPos.y > field.field_length/2. - field.penalty_area_length-500.) {
      blockingPos.y = field.field_length/2. - field.penalty_area_length-500.;
    }
    if (fabs(blockingPos.x) > field.field_width/2.) {
      blockingPos.x = (blockingPos.x/fabs(blockingPos.x)) * field.field_width/2.;
    }
      
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
      blockingPos += Vec((blockingPos.x < 0 ? -1 : 1) * 400., -700.); // vertrauen ist gut... TODO: Kontrolle...
    }
    
    // Ausweichposition berechnen, wenn zwischen aktueller Position und Ziel ein Mitspieler mit Ball ist
    
    if ((WBOARD->teamPossessBall() || WBOARD->teammateNearBall()) &&
        (ballLocation.pos_known == BallLocation::known ||
         ballLocation.pos_known == BallLocation::communicated) &&
        ballLocation.pos.y < robot.pos.y) {
      Quadrangle toPos(robot.pos, blockingPos, 1000);
      LOUT << "\% yellow thin solid " << toPos << endl;
      if (toPos.is_inside(ballLocation.pos.toVec())) {
        Frame2d quad2world( robot.pos, (blockingPos - robot.pos).angle() - Angle::quarter);
        Frame2d world2quad= quad2world;
        world2quad.invert();
        
        Vec relBallPos = world2quad * ballLocation.pos.toVec();
        Vec relAvoidPos; 
        if (relBallPos.x > 0) {
          relAvoidPos = relBallPos - Vec(2000.,0.);
        }
        else {
          relAvoidPos = relBallPos + Vec(2000.,0.);
        }
        Vec absAvoidPos = quad2world * relAvoidPos;
        if (fabs(absAvoidPos.x) > field.field_width/2 + 300) {
          absAvoidPos.x = (field.field_width/2. + 300) * absAvoidPos.x/fabs(absAvoidPos.x);
        }
        blockingPos = absAvoidPos;
        LOUT << "\% black solid thick " << Circle(blockingPos, 150) << endl;
      }
    }
    
    Skill* newSkill = skill;

    fast_skill->setTargetPos(blockingPos);
    skill->init(blockingPos, transVel, ballLocation.pos.toVec()-robot.pos);
    LOUT << "% yellow solid thick cross" << ballLocation.pos.x << " " << ballLocation.pos.y << endl;
    
   // bool drivingLaneFree= obstacle_distance_to_line_inside_field(robot.pos, robot.pos + (blockingPos- robot.pos).normalize() * 1500, MWM.get_obstacle_location(t)) < 500;
    
    //kann das boost behavior genutzt werden? -> Hindernisse checkt es selber, Abstand zum Ziel leider nicht.
    if (boostAllowed && ballInLane != BALL_MIDDLE && (blockingPos-robot.pos).length() > 2000. && robot.vtrans.length() > .7 &&
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
  BSupportNearBall::checkInvocationCondition(const Time& tt) throw()
  {
//    const BallLocation& ballLocation = MWM.get_ball_location(tt);  
//    const FieldGeometry& field = MWM.get_field_geometry(); 
    bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
   
    return 
      presentTactics != TACTICS_DEFENSIVE && (seheBall || seheKommuniziertenBall);
  }

  bool BSupportNearBall::checkCommitmentCondition(const Time& tt) throw()
  {
    return BSupportNearBall::checkInvocationCondition(tt);
  }
}

