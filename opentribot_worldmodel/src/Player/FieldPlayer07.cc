#include <cmath>
#include "FieldPlayer07.h"
#include "PlayerFactory.h"
#include "WhiteBoard.h"
#include "../Fundamental/geometry.h"
#include "../WorldModel/WorldModel.h"
#include "../WorldModel/FataMorgana.h"
#include "../Behavior/Skills/WithoutBall/SPatrol.h"
#include "../Behavior/SPBehavior.h"
#include "../Behavior/Behaviors/ApproachingBall/BComplexApproachBallFreePlay.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingToGoal.h"
#include "../Behavior/Behaviors/ApproachingBall/BVolleyApproach.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BEindhoven.h"
#include "../Behavior/Behaviors/BallHandling/BBreakAttack.h"
#include "../Behavior/Behaviors/BallHandling/BShakeOffDefender.h"
#include "../Behavior/Behaviors/BallHandling/BRetreatDribble.h"
#include "../Behavior/Behaviors/BallHandling/BWingAttack.h"
#include "../Behavior/Behaviors/BallHandling/BPassSpontaneously.h"
#include "../Behavior/Behaviors/BallHandling/BBefreiungsschlag.h"
#include "../Behavior/Behaviors/BallHandling/BTouchBallAfterStandard.h"
#include "../Behavior/Behaviors/BallHandling/BEigenMove.h"
#include "../Behavior/Behaviors/BallHandling/BStuckOwnsBall.h"
#include "../Behavior/Behaviors/BallHandling/BStuckDistanceShooter.h"
#include "../Behavior/Behaviors/BallHandling/BBoostBallToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BPassBeforeGoal.h"
#include "../Behavior/Behaviors/BasicMovements/BShootEmergency.h"
#include "../Behavior/Behaviors/BasicMovements/BShoot.h"
#include "../Behavior/Behaviors/BasicMovements/BDraufhalten.h"
#include "../Behavior/Behaviors/BasicMovements/BFarShoot.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/BasicMovements/BGotoPosEvadeObstacles.h"
#include "../Behavior/Behaviors/WithoutBall/BSupportLongPass.h"
#include "../Behavior/Behaviors/WithoutBall/BSupportNearBall.h"
#include "../Behavior/Behaviors/WithoutBall/BCounterAttack.h"
#include "../Behavior/Behaviors/WithoutBall/BPreventInterferenceWithPass.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/SpecialGameStates/BTestStateStop.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituationNew.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPostOpponentStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOwnIndirectStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BOwnPenalty.h"
//#include "../Behavior/Behaviors/StandardSituations/BOwnStandardSituation.h"
#include "../Behavior/Behaviors/ZonePressure/BZonePressure.h"
#include "../Behavior/Behaviors/WithoutBall/BLeaveGoal.h"
#include "../Behavior/Behaviors/WithoutBall/BOpposeBall.h"
#include "../Behavior/Behaviors/ApproachingBall/BInterceptBallStatic.h"
#include "../Behavior/Behaviors/ApproachingBall/BCatchBall.h"
#include "../Behavior/Behaviors/ApproachingBall/BTurnAroundPos.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallStatic.h"

// Innerhalb dieser Datei wird die spielertypspezifische Strategie des 
// Feldspielers festgelegt. Die vom Feldspieler verwendeten Behaviors und
// Skills sollten soweit wie m�lich generisch gehalten werden, d.h. 
// Strategiespezifische Einstellungen wie Aktionsbereiche und 
// Rollenspezifikationen sollten aussschlie�ich hier ber das setzen von 
// allgemeinen Parametern der Behaviors und durch Ableiten und �erschreiben 
// ihrer Aktivierungsbedingungen vorgenommen werden.

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (
        string("Feldspieler07"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, 
			    PlayerType*) 
      throw (TribotsException,std::bad_alloc) 
    {
      return new FieldPlayer07 (reader);
    }
  };
  Builder the_builder;
}

namespace Tribots {   // unbenannter Namespace notwenig, um Verwechslungen beim Linken mit FieldPlayer zu vermeiden

static string FieldPlayer07_role = "ballL";


//
// BPatrol verwendet den Skill SPatrol mit geeigneten Patrolpositionen,
// die von der aktuellen Rolle abgeleitet werden.
//
class BPatrolFP07 : public Behavior {
public:
 
  BPatrolFP07() : Behavior("BPatrolFP07"), patrol(new SPatrol()) {};
  ~BPatrolFP07() throw() { delete patrol; }
  
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);

    return (ball.pos_known == BallLocation::unknown); 
  }
  virtual bool checkInvocationCondition(const Time& t) throw() {
    return BPatrolFP07::checkCommitmentCondition(t);
  }
  virtual DriveVector getCmd(const Time& t) throw(TribotsException) {
    const FieldGeometry& field = MWM.get_field_geometry();      
      
    if (FieldPlayer07_role == "ballL")  {
      if (WBOARD->onlyOneRobot()) {
        std::vector<Vec> positions;
        positions.push_back(Vec(-2*field.center_circle_radius,-field.center_circle_radius));
        positions.push_back(Vec(-field.center_circle_radius, -2*field.center_circle_radius));
        positions.push_back(Vec(field.center_circle_radius, -2*field.center_circle_radius));
        positions.push_back(Vec(2*field.center_circle_radius, -field.center_circle_radius));
        patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
        patrol->setPatrolPositions(positions);
      }else if(WBOARD->onlyThreeRobots()||WBOARD->onlyTwoRobots()) {
        std::vector<Vec> positions;
        positions.push_back(Vec(-2*field.center_circle_radius, field.center_circle_radius));
        positions.push_back(Vec(-field.center_circle_radius, 2*field.center_circle_radius));
        positions.push_back(Vec(field.center_circle_radius, 2*field.center_circle_radius));
        positions.push_back(Vec(2*field.center_circle_radius, field.center_circle_radius));
        patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
        patrol->setPatrolPositions(positions);
      }else {
        std::vector<Vec> positions;
        positions.push_back(Vec(-2*field.center_circle_radius, 2*field.center_circle_radius));
        positions.push_back(Vec(-2*field.center_circle_radius, field.center_circle_radius));
        positions.push_back(Vec(-500., 1.5*field.center_circle_radius));
        patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
        patrol->setPatrolPositions(positions);
      }
    }
    else if (FieldPlayer07_role == "ballR")  {
      std::vector<Vec> positions;
      positions.push_back(Vec(+2*field.center_circle_radius, 2*field.center_circle_radius));
      positions.push_back(Vec(+2*field.center_circle_radius, field.center_circle_radius));
      positions.push_back(Vec(+500., 1.5*field.center_circle_radius));
      patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
      patrol->setPatrolPositions(positions);
    }
    else if (FieldPlayer07_role == "left") {
      patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
      patrol->setPatrolPositions(Vec(-field.penalty_area_width/2,
                                     -field.field_length/2+field.penalty_area_length),
                                 Vec(-field.penalty_area_width/2, -1000.));
    }
    else if (FieldPlayer07_role == "right") {
      patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
      patrol->setPatrolPositions(Vec(field.penalty_area_width/2,
                                     -field.field_length/2+field.penalty_area_length),
                                 Vec(field.penalty_area_width/2, -1000.));
    }
    else { // safety
      if(WBOARD->onlyTwoRobots()){
        std::vector<Vec> positions;
        positions.push_back(Vec(0,-field.field_length/2+field.penalty_marker_distance));
        positions.push_back(Vec(-field.field_width*2/6,0));
        positions.push_back(Vec(field.field_width*2/6,0));
        patrol->setPatrolPositions(Vec(0.,0.),Vec(0.,0.));
        patrol->setPatrolPositions(positions);
      }else{
        patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel n�tig TODO: remove this constructor
        patrol->setPatrolPositions(Vec(field.penalty_area_width/2,
                                     -field.field_length/2+field.penalty_area_length),
                                 Vec(-field.penalty_area_width/2., -field.field_length/2+field.penalty_area_length));
      }
    }
    
    return patrol->getCmd(t);
  }
  
protected:
  SPatrol* patrol;
};

//
// BStuckOwnsBall abgeleitet fuer Verwendung in Feldspieler 
//
class BStuckOwnsBallConditionedFP07 : public BStuckOwnsBall {
public:
  BStuckOwnsBallConditionedFP07() : BStuckOwnsBall(), act(true), off(false)
  { name = "BStuckOwnsBallConditionedFP07"; }
  virtual bool checkInvocationCondition(const Time& t) throw() {
    if (off) return false;
    const RobotLocation& robot = MWM.get_robot_location(t);
    return 
      BStuckOwnsBall::checkInvocationCondition(t) && 
      (robot.pos.y > 500. ||
       act ||
       MWM.get_game_state().refstate == preOwnThrowIn ||
       MWM.get_game_state().refstate == preOwnGoalKick ||
       MWM.get_game_state().refstate == preOwnCornerKick ||
       MWM.get_game_state().refstate == preOwnFreeKick ||
       MWM.get_game_state().refstate == preOwnKickOff);
   }
   void updateTactics (const TacticsBoard& tb) throw () {
     if (tb[string("KampfUmBall")]==string("BallFreispielen")) {
       act=true;  off = false;
     }
     else if (tb[string("KampfUmBall")]==string("TorAbdecken"))     {
       act=false; off = false;
     }
     else { // AuchImVorfeldVorsichtig | NurEngAusweichen
       act=false; off = true;
     }
   }
protected:
  bool act;
  bool off;
};
  
class BPreventInterferenceWithPassConditioned : public BPreventInterferenceWithPass {
public:
  BPreventInterferenceWithPassConditioned(const std::string name= "BBPreventInterferenceWithPassConditioned") 
  : BPreventInterferenceWithPass(name) {}
  virtual bool checkInvocationCondition(const Time& t) throw() {
    string msg = MWM.get_message_board().scan_for_prefix("stayAwayFromPassingLane!"); // this word is sent to all but the receiver of a querpass
    const RobotLocation& robot = MWM.get_robot_location(t);
    const FieldGeometry& fgeom = MWM.get_field_geometry();
    bool robotMayInterfere = 
      fabs(robot.pos.x) < fgeom.penalty_area_width &&
      robot.pos.y < fgeom.field_length/2. - fgeom.penalty_area_length &&
      robot.pos.y > 0;
    return
      !WBOARD->doPossessBall(t) &&
      BPreventInterferenceWithPass::checkInvocationCondition(t) &&
      msg != "" &&
      robotMayInterfere;
  }
};      

class BCounterAttackConditioned : public BCounterAttack {
public:
  BCounterAttackConditioned() : BCounterAttack(6000) 
  { name = "BCounterAttackConditioned";}
  
  virtual bool checkInvocationCondition(const Time& t) throw() {
    WBOARD->checkMessageBoard();
    return 
	    BCounterAttack::checkInvocationCondition(t) &&//Anfangsbedingungen 
	    WBOARD->doCounterAttack() &&                 // "NachVorne" gehoert
	    MWM.get_game_state().refstate == freePlay && // freies spiel
	    MWM.get_robot_location(t).pos.y < 1000.;        // noch in eigener haelfte
	    
   }
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const RobotLocation& robot = MWM.get_robot_location(t);
    const BallLocation& ball = MWM.get_ball_location(t);	
    return 
      BCounterAttack::checkCommitmentCondition(t) && 
	    MWM.get_game_state().refstate == freePlay &&
      (MWM.get_ball_location(t).pos_known == BallLocation::unknown ||
       MWM.get_ball_location(t).pos_known == BallLocation::raised ||
	     (robot.pos.y+500. < ball.pos.y || ball.velocity.y > 1.)) &&
	    (ball.pos.toVec()-robot.pos).length() > 1500.; 
  }   
   
protected:
};

class BVolleyApproachConditioned : public BVolleyApproach {
public:
  BVolleyApproachConditioned() {
    name = "BVolleyApproachConditioned";
  }
  bool checkInvocationCondition(const Time& t) throw () {
    const FieldGeometry& field = MWM.get_field_geometry();
    WBOARD->checkMessageBoard();
    XYRectangle actArea (Vec(-field.penalty_area_width/2., field.field_length/2.),
                         Vec(field.penalty_area_width/2., field.field_length/2.-2*field.penalty_area_length));
    bool condition =  volley_probability > 0.01 && WBOARD->receivePass() &&
                      actArea.is_inside(MWM.get_ball_location(t).pos.toVec()) &&
                      checkGeometricCondition(t, true);
    if (condition) LOUT << "checked BVolleyApproachConditioned. may be executed." << endl;
    return condition;
  }
};

class BVolleyApproachAfterOwnSetPlay : public BVolleyApproach {
protected:
  Time timeLatestSetPlay;
public:
  BVolleyApproachAfterOwnSetPlay() {
    name = "BVolleyApproachAfterOwnSetPlay";
  }
  bool checkInvocationCondition(const Time& t) throw () {
    return t.diff_msec (timeLatestSetPlay)<5000 && BVolleyApproach::checkInvocationCondition(t);
  }
  void cycleCallBack(const Time& t) throw ()
  {
//    LOUT << "CCB\n";
    const BallLocation& ball (MWM.get_ball_location(t));
    if ((MWM.get_game_state().refstate==preOwnThrowIn ||
        MWM.get_game_state().refstate==preOwnFreeKick) &&
        ball.pos_known==BallLocation::known &&
        ball.pos.y>-1000)
      timeLatestSetPlay=t;
    volley_decision=true;
    // BVolleyApproach::cycleCallBack(t) nicht aufrufen, da volley_decision hier immer true sein soll
  }
};

//
// BStuckStandardSituation
//
class BStuckStandardSituationFP07 : public BStuckOwnsBall {
public:
  BStuckStandardSituationFP07() : BStuckOwnsBall(), act(true)
  { name = "BStuckStandardSituationFP07"; }
  virtual bool checkInvocationCondition(const Time& t) throw() {
    return 
      act && BStuckOwnsBall::checkInvocationCondition(t) && 
      (MWM.get_game_state().refstate == preOwnThrowIn ||
       MWM.get_game_state().refstate == preOwnGoalKick ||
       MWM.get_game_state().refstate == preOwnCornerKick ||
       MWM.get_game_state().refstate == preOwnFreeKick ||
       MWM.get_game_state().refstate == preOpponentFreeKick ||
       MWM.get_game_state().refstate == preOpponentThrowIn ||
       MWM.get_game_state().refstate == preOpponentCornerKick ||
       MWM.get_game_state().refstate == preOpponentGoalKick ||
       MWM.get_game_state().refstate == preOpponentKickOff ||
       MWM.get_game_state().refstate == preOwnFreeKick ||
       MWM.get_game_state().refstate == preOwnKickOff);
  }
  void updateTactics (const TacticsBoard& tb) throw () {
    if (tb[string("StuckStandards")]==string("aus")) {
      act=false;
    }
    else {
      act=true;
    }
  }
protected:
  bool act;
};

/*
class BBallTurning: public BDIBehavior {
   public:
      BBallTurning(): BDIBehavior("BBallTurning") {
         addOption(new BApproachBallStatic());
         addOption(new BTurnAroundPos());
      }
      
      ~BBallTurning() throw() {}

      virtual bool checkInvocationCondition(const Time& texec) throw() {
      const RobotLocation& robot_exec (MWM.get_robot_location (texec));
      const BallLocation& ball_exec (MWM.get_ball_location (texec));

      if (ball_exec.pos_known == BallLocation::unknown) {
        return false;
      }

      if (fabs( (robot_exec.pos-ball_exec.pos.toVec()).length()) > 1000)
      {
         LOUT << "BBallTurning: Ball is too far away";
         return false;
      }
      Angle robot_ball = (ball_exec.pos.toVec()-robot_exec.pos).angle()-
                        (robot_exec.heading+Angle::quarter);
      Vec relBall = WBOARD->getAbs2RelFrame(texec) * ball_exec.pos.toVec();
      if (fabs(robot_ball.get_deg_180()) > 30 && 
         !(fabs(relBall.x) < 500. && relBall.y > 0 && relBall.y < 700.)) {
        LOUT << "BBallTurning: not approaching, angle to big and not very close to robot:" << robot_ball.get_deg_180() << endl;
        return false;
      }


      //LOUT << "BBallTurnin: angleofball="<< fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) << endl;
      //LOUT << "BBallTurnin: 1st comp. (no fabs):"<< ((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180() << endl;
      //LOUT << "BBallTurnin: 2nd comp. (no quarter):"<< ((ball_exec.velocity.toVec() / robot_exec.heading).angle()).get_deg_180() << endl;
      //LOUT << "BBallTurnin: 3rd comp. (no heading):"<< ((ball_exec.velocity.toVec()).angle()).get_deg_180() << endl;

     // Es ist ok, wenn sich der ball langsam auf den Roboter zubewgt. Bei hoeheren Geschwindigkeiten muss er sich aber vom ROboter
     // entfernen (abgeprallt sein).
     if(fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) > 90 &&
        ball_exec.velocity.length() > .7) {
       LOUT << "BBallTurning not invoced, because the ball is moving towards the robot\n";
       LOUT << "angle         : "<< fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) <<"\n";
       return false;
     }
      return BDIBehavior::checkInvocationCondition(texec); // generisch, checkt ob es Behaviors gibt die die Kontrolle haben wollen
      
     }


      virtual bool checkCommitmentCondition(const Time& texec) throw() {
         const RobotLocation& robot_exec (MWM.get_robot_location (texec));
         const BallLocation& ball_exec (MWM.get_ball_location (texec));

         if (ball_exec.pos_known == BallLocation::unknown) {
           return false;
         }

         if (fabs( (robot_exec.pos-ball_exec.pos.toVec()).length()) > 1100)
         {
            LOUT << "BBallTurning: Ball is too far away";
            return false;
         }
         Angle robot_ball = (ball_exec.pos.toVec()-robot_exec.pos).angle()-
                           (robot_exec.heading+Angle::quarter);
         Vec relBall = WBOARD->getAbs2RelFrame(texec) * ball_exec.pos.toVec();
         if (fabs(robot_ball.get_deg_180()) > 35 && 
            !(fabs(relBall.x) < 550. && relBall.y > 0 && relBall.y < 750.)) {
           LOUT << "BBallTurning: not approaching, angle to big and not very close to robot:" << robot_ball.get_deg_180() << endl;
           return false;
         }
        // Es ist ok, wenn sich der ball langsam auf den Roboter zubewgt. Bei hoeheren Geschwindigkeiten muss er sich aber vom ROboter
        // entfernen (abgeprallt sein).
        if(fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) > 90 &&
           ball_exec.velocity.length() > .7) {
          LOUT << "BBallTurning not invoced, because the ball is moving towards the robot\n";
          LOUT << "angle         : "<< fabs(((ball_exec.velocity.toVec() / robot_exec.heading).angle() - Angle::quarter).get_deg_180()) <<"\n";
          return false;
        }
         return BDIBehavior::checkCommitmentCondition(texec); // generisch, checkt ob es Behaviors gibt die die Kontrolle haben wollen
      }
};

*/


class BallPassingReceiver : public SPBehavior {
private:

   Time timeout;
public:
  virtual bool checkInvocationCondition(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);    
    if (ball.pos_known == BallLocation::unknown) { 
        LOUT << "BallPassingReceiver: Sorry, I can't see the ball. /n";
      return false;
    }
    return SPBehavior::checkInvocationCondition(t);
  }
  
  /**
   */
  BallPassingReceiver () 
    : SPBehavior("FieldPlayer07BallPassingReceiver")
  {
     appendStage(new BOpposeBall(), false, true);
     appendStage(new BInterceptBallStatic(), false, true);
     // let the usual Behaviors do the work
     // appendStage(new BBallTurning(), false, true);
     timeout.update();
  }
  
  ~BallPassingReceiver() throw () {
  }

  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);    
    if (ball.pos_known == BallLocation::unknown) { // TODO: CHECK
      LOUT << "BallPassingReceiver: Sorry, I can't see the ball. /n";
      return false;
    }

    //LOUT << "DEBUG SPBallPassingReceiver: timeout=" << timeout.elapsed_sec() << endl;
    if (timeout.elapsed_sec() > 20) {
      // LOUT << "SPBallPassingReceiver: timeout" << endl;
      return false;
    }
    return SPBehavior::checkCommitmentCondition(t);  
  }

  virtual void gainControl(const Time& t) throw() {
      //LOUT << "DEBUG SPBallPassingReceiver GAIN CONTROL: timeout=" << timeout.elapsed_sec() << endl;
      timeout.update();
  }
  
};  // BallPassingReceiver finished 



// Ballstack - geht immer an, wenn der Spieler zum Ball darf
//
class BallOwnerStack : public BDIBehavior {
private:
  Area* ballLeftActionArea; ///< Aktivitaetsbereich des linken Ballspielers
  Area* ballRightActionArea;///< Aktivitaetsbereich des rechten Ballspielers
  Area* safetyActionArea;///< Aktivitaetsbereich des Ausputzers
  Area* leftActionArea; ///< Aktivitaetsbereich des linken Verteidigers
  Area* rightActionArea;///< Aktivitaetsbereich des rechten Verteidigers
  int counter;          ///< zur Ueberbrueckung weniger Zyklen ohne Ballbesitz
  Time _lastPassTime;   ///< Wann wurde die letzte Passentscheidung getroffen?
  bool _receivePass;    ///< Soll dieser Spieler gerade einen Pass annehmen?
  Time lastRoleRequest; ///< Wann wurde zuletzt eine neue Rolle erbeten?
  bool obeyPenaltyAreaRules;///< An die nur-1-Spieler-im-Strafraum Regel halten?

  /**
   * Ueberpruefung allgemeiner Bedingungen.
   */
  virtual bool checkConditions(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);
    WBOARD->checkMessageBoard();  // brauche Infos aus dem Messageboard
    
    if ((WBOARD->teamPossessBall() || 
         (WBOARD->teammateNearBall() && WBOARD->getZonePressureRole() != "ballL" && WBOARD->getZonePressureRole() != "ballR")) && 
        ! _receivePass) {
      return false;
    }
    if (WBOARD->teammateNearBall() && ! _receivePass) {  // TODO: check, ob vielleicht doch hinfahren soll????
      return false;
    }
    if (ball.pos_known == BallLocation::unknown || 
        ball.pos_known == BallLocation::raised) { // TODO: CHECK
      return false;
    }
    return true;
  }
  
public:
  /**
   * Konstruktor des BallStacks.
   *
   * \param leftActionArea Aktivitaetsbereich des linken Verteidigers
   * \param rightActionArea Aktivitaetsbereich des rechten Verteidigers
   * \param opponent_type Typ des Angreifers. Vorlaeufige Methode eine 
   *                      Strategie zu spezifizieren.
   */
  BallOwnerStack(const Area& ballLeftActionArea,
                 const Area& ballRightActionArea,
                 const Area& leftActionArea,
                 const Area& rightActionArea,
                 const Area& safetyActionArea,
                 int longPassKickDuration,
                 int hackKickLength,
                 bool obeyPenaltyAreaRules=true) 
    : BDIBehavior("FieldPlayer07BallOwnerStack"), 
      ballLeftActionArea(ballLeftActionArea.clone()), 
      ballRightActionArea(ballRightActionArea.clone()), 
      safetyActionArea(safetyActionArea.clone()), 
      leftActionArea(leftActionArea.clone()), 
      rightActionArea(rightActionArea.clone()), counter(0),
      obeyPenaltyAreaRules(obeyPenaltyAreaRules)
  {
    addOption (new BShootEmergency(hackKickLength));
    addOption (new BFarShoot(.5));
    addOption (new BDraufhalten(.25, hackKickLength));
    addOption (new BShoot(hackKickLength));
    addOption (new BBefreiungsschlag());
    addOption (new BStuckOwnsBallConditionedFP07());
    addOption (new BPassBeforeGoal(longPassKickDuration));
    addOption (new BPassSpontaneously()); 
    addOption (new BStuckDistanceShooter());
    addOption (new BTouchBallAfterStandard());
    addOption (new BallPassingReceiver());
    addOption (new BEindhoven());
//    addOption (new BBreakAttack());
    addOption (new BEigenMove());
    addOption (new BRetreatDribble());
    addOption (new BShakeOffDefender());
    addOption (new BWingAttack(2.5, 1., true));
    addOption (new BBoostBallToGoal());
    addOption (new BDribbleBallStraightToGoalEvadeSidewards());


    addOption (new BDribbleBallToGoal());

 // addOption (new BVolleyApproachConditioned()); // nach paessen vor dem Tor an, unabhaengig von gesetzter Wk.
 // addOption (new BVolleyApproach());
   
 addOption (new BComplexApproachBallFreePlay());
    lastRoleRequest.update();   
  }
  
  ~BallOwnerStack() throw () {
    delete ballLeftActionArea;
    delete ballRightActionArea;
    delete safetyActionArea;
    delete leftActionArea;
    delete rightActionArea;
  }

  /**
   * Generiert einen DriveVector. Verwendet dazu die unveraenderte Methode
   * des BDI-Verhaltens (siehe BDIBehavior), ueberprueft aber vorher, ob
   * beim Teamcontrol die Rolle "ball" aktiv beantragt werden soll.
   */
  virtual DriveVector getCmd(const Tribots::Time& t) throw (Tribots::TribotsException) { 
    // Anfrage an das teamcontrol, Rolle "ball" zu bekommen.
    const RobotLocation& robot = MWM.get_robot_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    if (FieldPlayer07_role != "ballL" && FieldPlayer07_role != "ballR" && WBOARD->doPossessBall(t) 
        && t.diff_sec(lastRoleRequest) > 1) // nicht zu haeufig anfragen
    { 
      MWM.get_message_board().publish("request_role_ball");
      LOUT << "request_role_ball send" << endl;
      lastRoleRequest = t;
    }
    // schauen, ob der stuermer auf falscher seite ist. wenn ja, rollenwechsel beantragen (links mit rechts tauschen). Idee: Unterstuetzender, zweiter Stuermer kreuzt hinter dem Ballfuehrenden Spieler
    if (WBOARD->doPossessBall(t) &&
        ((FieldPlayer07_role == "ballL" && robot.pos.x > +field.goal_area_width/2. && !WBOARD->onlyOneRobot() && !WBOARD->onlyTwoRobots() && !WBOARD->onlyThreeRobots()) || 
         (FieldPlayer07_role == "ballR" && robot.pos.x < -field.goal_area_width/2.)) && 
        t.diff_sec(lastRoleRequest) > 1)    // nicht zu haeufig anfragen
    {  
      MWM.get_message_board().publish("request_switch_lr");
      LOUT << "request_switch_LR send" << endl;
      lastRoleRequest = t;
    }
    return BDIBehavior::getCmd(t);
  }

  virtual void cycleCallBack(const Time& t) throw() {
    BDIBehavior::cycleCallBack(t);
    WBOARD->checkMessageBoard();
    unsigned int robotSelfId = MWM.get_robot_id(); 
    bool rp = WBOARD->receivePass() || WBOARD->receiveSetPlayShortPass();
    if (rp) { 
      LOUT << "in cycleCallBack of BallStack: heard 'receive_pass'" << endl;
    }
    bool rpp = WBOARD->receivePlannedPass(robotSelfId);
    string msg = MWM.get_message_board().scan_for_prefix("pass:");
    if (msg != "") {
      LOUT << "pass gefunden: " << msg << endl;
      istringstream str(msg);
      string tmp;
      unsigned int number;
      str >> tmp >> number;
      if (str && MWM.get_robot_id() == number) {              // passempfaenger wurde kommuniziert
        LOUT << "Das bin ja ich!" << endl;
        if (!rp) LOUT << "obwhohl kein receive_pass gefunden, anschalten" << endl;
        rp = true;
      }
    }
    if (rpp)
      LOUT << "in cycleCallBack of BallStack: heard 'receive_planed_pass'" << endl;

    if (rp||rpp) {
      _lastPassTime = t;
      _lastPassTime.add_sec(4);
      _receivePass = true;
    }
    if (_lastPassTime < t) {
      _receivePass = false;
    }
   if (WBOARD->detectPassedBall(t)) {
      LOUT << "PassDetector cycleCallBack(): ball passed" << endl;
      _receivePass = true;
   }
  }



  virtual bool checkCommitmentCondition(const Time& t) throw() {
    if (!checkConditions(t)) { // auf jeden fall aufhoeren, wenn nicht erfuellt
      return false;
    }
    
    if (WBOARD->doPossessBall(t)) { // bei Ballbesitz weiter machen...
      counter = 30;  // 30 Zyklen weitermachen, auch wenn ballbesitz weg ist
      return true;
    }
    // kein ballbesitz!
    if (counter > 0) {
      LOUT << "BallStack: Den " << 31-counter << ". Zyklus keinen Ballbesitz "
	   << "mehr." << endl;
      --counter;
    }
    const BallLocation& ball = MWM.get_ball_location(t);
    if ((!(FieldPlayer07_role == "ballL" && 
           (WBOARD->onlyTwoRobots() || WBOARD->onlyThreeRobots() || 
            ballLeftActionArea->is_inside(ball.pos.toVec()))) &&  // zum ball nur, wenn ballspieler
         !(FieldPlayer07_role == "ballR" && ballRightActionArea->is_inside(ball.pos.toVec())) &&
         !(FieldPlayer07_role == "left" && // oder linker spieler und 
            leftActionArea->is_inside(ball.pos.toVec())) && // ball ist links
         !(FieldPlayer07_role == "right" && // oder rechter spieler und
            rightActionArea->is_inside(ball.pos.toVec())) && // ball ist rechts
         !(FieldPlayer07_role =="safety" && safetyActionArea->is_inside(ball.pos.toVec()))
         ) && counter <= 0 && 
        !WBOARD->onlyOneRobot() &&
        !_receivePass) {       // DIESE ZEILE NEU FUER WM2008
                               // 30 zyklen nach ballverlust auch ausserhalb
                               // des 
      return false;            // zustaendigkeitsbereichs fahren
    }
    // Der ball-spieler darf in die strafraumarea rein, waehrend die beiden 
    // pong spieler draussen bleiben muessen, wenn sich an die regeln gehalten
    // wird
    const FieldGeometry& field = MWM.get_field_geometry();
    XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - 200, 
                                -field.field_length / 2. - 800), // hinter gl
                            Vec(field.penalty_area_width/2. + 200,
                                -field.field_length / 2. + 200 +
                                field.penalty_area_length));
    XYRectangle oppPenaltyArea(Vec(-field.penalty_area_width/2. - 200, 
                                   +field.field_length / 2. + 800), // hinter gl
                               Vec(field.penalty_area_width/2. + 200,
                                   +field.field_length / 2. - 200 -
                                   field.penalty_area_length));
    if (FieldPlayer07_role != "ballL" &&
        FieldPlayer07_role != "ballR" &&    // TODO: da duerfen noch 2 rein!
        FieldPlayer07_role != "safety" &&
        penaltyArea.is_inside(ball.pos.toVec())) { // nicht an, wenn der Ball im Strafraum
      return false;
    }
    if (!WBOARD->doPossessBall(t) &&
        WBOARD->getActiveRobots() == 5 &&
        FieldPlayer07_role != "safety" &&
        penaltyArea.is_inside(ball.pos.toVec())) {
      return false;
    }  
    if (!WBOARD->doPossessBall(t) &&
        WBOARD->getActiveRobots() >= 4 &&
        (FieldPlayer07_role == "ballL" || FieldPlayer07_role == "ballR") &&
        WBOARD->teammateNearBall() &&
        (oppPenaltyArea.is_inside(ball.pos.toVec()) ||
         penaltyArea.is_inside(ball.pos.toVec()))) {
      return false;
    }
    if (!WBOARD->doPossessBall(t) &&
        WBOARD->getActiveRobots() >= 4 &&
        ((FieldPlayer07_role == "ballL" && ball.pos.x >  300.) || 
         (FieldPlayer07_role == "ballR" && ball.pos.x < -300.)) &&
        oppPenaltyArea.is_inside(ball.pos.toVec())) {
      return false;
    }
    return true;   // im zust�digkeitsbereich, oder in den letzten 20 zyklen
  }                // mal den ball gehabt

  virtual bool checkInvocationCondition(const Time& t) throw() {
    if (!checkConditions(t)) {
      return false;
    }
    LOUT << "After checkConditions" << endl;
    const BallLocation& ball = MWM.get_ball_location(t);
    //cerr << "ballLeftActionArea: " << ballLeftActionArea->is_inside(ball.pos.toVec()) << endl;
    if ((!(FieldPlayer07_role == "ballL" && 
           (WBOARD->onlyTwoRobots() || WBOARD->onlyThreeRobots() || 
            ballLeftActionArea->is_inside(ball.pos.toVec()))) &&  // zum ball nur, wenn ballspieler
         !(FieldPlayer07_role == "ballR" && ballRightActionArea->is_inside(ball.pos.toVec())) &&
         !(FieldPlayer07_role == "left" && // oder linker spieler und 
           leftActionArea->is_inside(ball.pos.toVec())) && // ball ist links
         !(FieldPlayer07_role == "right" && // oder rechter spieler und
           rightActionArea->is_inside(ball.pos.toVec())) && // ball ist rechts
         !(FieldPlayer07_role =="safety" && safetyActionArea->is_inside(ball.pos.toVec()))
         ) &&  
        !WBOARD->onlyOneRobot() &&
        !WBOARD->doPossessBall(t) &&
        !_receivePass) // DIESE ZEILE NEU FUER WM2008
    {
      return false;
    }
    // Der ball-spieler darf in die strafraumarea rein, waehrend die beiden 
    // pong spieler draussen bleiben muessen, wenn sich an die regeln gehalten
    // wird
    const FieldGeometry& field = MWM.get_field_geometry();
    XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - 200, 
                                -field.field_length / 2. - 800), // hinter gl
                            Vec(field.penalty_area_width/2. + 200,
                                -field.field_length / 2. + 200 +
                                field.penalty_area_length));
    XYRectangle oppPenaltyArea(Vec(-field.penalty_area_width/2. - 200, 
                                   +field.field_length / 2. + 800), // hinter gl
                               Vec(field.penalty_area_width/2. + 200,
                                   +field.field_length / 2. - 200 -
                                   field.penalty_area_length));
    if (!WBOARD->doPossessBall(t) &&
        obeyPenaltyAreaRules && FieldPlayer07_role != "ballL" &&
        FieldPlayer07_role != "ballR" &&    // TODO: da duerfen noch 2 rein!
        FieldPlayer07_role != "safety" && 
        penaltyArea.is_inside(ball.pos.toVec())) { // nicht an, wenn der Ball im Strafraum
      return false;
    }
    if (!WBOARD->doPossessBall(t) &&
        WBOARD->getActiveRobots() == 5 &&
        FieldPlayer07_role != "safety" &&
        penaltyArea.is_inside(ball.pos.toVec())) {
      return false;
    }  
    if (!WBOARD->doPossessBall(t) &&
        WBOARD->getActiveRobots() >= 4 &&
        (FieldPlayer07_role == "ballL" || FieldPlayer07_role == "ballR") &&
        WBOARD->teammateNearBall() &&
        (oppPenaltyArea.is_inside(ball.pos.toVec()) ||
         penaltyArea.is_inside(ball.pos.toVec()))) {
      return false;
    }
    if (!WBOARD->doPossessBall(t) &&
        WBOARD->getActiveRobots() >= 4 &&
        ((FieldPlayer07_role == "ballL" && ball.pos.x >  300.) || 
         (FieldPlayer07_role == "ballR" && ball.pos.x < -300.)) &&
        oppPenaltyArea.is_inside(ball.pos.toVec())) {
      return false;
    }
    return true;
  }  

  virtual void loseControl(const Time&) throw(TribotsException) 
  { counter = 0; }
};   


static const char *_tribots_fp07_roles[5] = { "ballL", "ballR", "left", "right", "safety" };

bool
FieldPlayer07::set_role(const char* role) throw ()
{
  if (BehaviorPlayer::set_role(role)) {
    FieldPlayer07_role = std::string(role);      // fuer "interne" behaviors
    WBOARD->setZonePressureRole(std::string(role));
    return true;
  }
  return false;
}

class BFP07Update : public Behavior {
public:
  int helpcounter;
  unsigned int rotate; ///< Mindestanzahl der Roboter, ab der in der Verteidigung rotiert wird.

  BFP07Update() : Behavior("BFP07Update"), rotate(4)
  {
	helpcounter = 60;
  }
  
  void updateTactics (const TacticsBoard& tb) throw () {
    cerr << "BFP07Upate: received updateTactics" << endl;
    if (tb[string("Rotation")]==string("Ab4Spielern")) {
      rotate = 4;
    }
    else if (tb[string("Rotation")]==string("Ab3Spielern")) {
      rotate = 3;
    }
    else {
      rotate = 99;
    }
  }

  virtual bool checkInvocationCondition(const Time& t) throw() {
    return false;
  }
  
  virtual void cycleCallBack(const Time& t) throw() {
    WBOARD->checkMessageBoard();
    const BallLocation& ball = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    if (!WBOARD->teamPossessBall() && !WBOARD->doPossessBall(t) &&
        ball.pos_known != BallLocation::unknown && 
        ball.pos_known != BallLocation::raised &&
        MWM.get_game_state().refstate == freePlay) {
      
	  if (helpcounter <=0 && WBOARD->getActiveRobots() >= rotate) {
	  
  	    const RobotLocation& robot = MWM.get_robot_location(t);
        		
		if ((WBOARD->getActiveRobots() ==3 || FieldPlayer07_role == "ballR") &&
          // right darf nicht rechts ueberholt werden
           (ball.pos.y > field.penalty_area_length + 700.) &&
           ((robot.pos.x < ball.pos.x-100. &&
            ball.pos.y-robot.pos.y < - 800.) ||
           (robot.pos.x < ball.pos.x &&
            ball.pos.y-robot.pos.y < -200. &&
            ball.velocity.y < robot.vtrans.y - 1.)) // zu schnell richtung eigenes tor
          ) {
          MWM.get_message_board().publish("helpright!");
          LOUT << "help right gerufen" << endl;
          LOUT << "\n% red word " << Vec(500,0) + robot.pos <<"help\n" ;
  		  helpcounter = 60; //naechste 60 zyklen keine hilferufe mehr erlauben
	    }
		
		if (FieldPlayer07_role == "ballL" && // left darf nicht links ueberholt werden
           (ball.pos.y > field.penalty_area_length + 700.) &&
           ((robot.pos.x > ball.pos.x+100. &&
            ball.pos.y-robot.pos.y < - 800.) ||
           (robot.pos.x > ball.pos.x &&
            ball.pos.y-robot.pos.y < -200. &&
            ball.velocity.y < robot.vtrans.y - 1.)) 
		   ) {
          MWM.get_message_board().publish("helpleft!");
          LOUT << "help left gerufen" << endl;
          LOUT << "\n% red word " << Vec(500,0) + robot.pos <<"help\n" ;
  		  helpcounter = 60; //naechste 60 zyklen keine hilferufe mehr erlauben
	    }
	  }	else {
		helpcounter = --helpcounter < 0 ? 0 : helpcounter; 
            // Ueberschlag vermeiden, gibt ja schon eine Menge zyklen ;-)
      }
    }
    
    LOUT << "Anzahl Roboter: " << WBOARD->getActiveRobots()
         << " only1: " << (WBOARD->onlyOneRobot()?"true":"false")
         << " only2: " << (WBOARD->onlyTwoRobots()?"true":"false")
         << " only3: " << (WBOARD->onlyThreeRobots()?"true":"false")
         << " only4: " << (WBOARD->onlyFourRobots()?"true":"false")
         << endl;

    // nun noch die rolle in standardsituationen setzen
    int stdRole = WhiteBoard::STANDARD_ROLE_A;
    
    // A-> Towards Ball on left   / block on right
    // B-> Towards Ball on right  / block on left
    // C-> execute on left  / catch on right
    // D-> execute on right / catch on right
    // E-> safety, stay away in middle of field (may execute goal kicks)
    
    if (WBOARD->onlyOneRobot()) {
      stdRole = WhiteBoard::STANDARD_ROLE_ABCDE;
    }
    else if (WBOARD->onlyTwoRobots()) {
      if (FieldPlayer07_role == "ballL" ||
          FieldPlayer07_role == "ballR") {
        stdRole = WhiteBoard::STANDARD_ROLE_AB;
      }
      else {
        stdRole = WhiteBoard::STANDARD_ROLE_CDE;
      }
    }
    else if (WBOARD->onlyThreeRobots()) {
      if (FieldPlayer07_role == "ballL" ||
          FieldPlayer07_role == "ballR") {
        stdRole = WhiteBoard::STANDARD_ROLE_AB;
      }
      else if (FieldPlayer07_role == "left") {
        stdRole = WhiteBoard::STANDARD_ROLE_C;
      }
      else {
        stdRole = WhiteBoard::STANDARD_ROLE_DE;
      }
    }
    else if (WBOARD->onlyFourRobots()) {
      if (FieldPlayer07_role == "ballL") {
        stdRole = WhiteBoard::STANDARD_ROLE_A;
      }
      else if (FieldPlayer07_role == "ballR") {
        stdRole = WhiteBoard::STANDARD_ROLE_B;
      }
      else if (FieldPlayer07_role == "left") {
        stdRole = WhiteBoard::STANDARD_ROLE_C;
      }
      else {
        stdRole = WhiteBoard::STANDARD_ROLE_DE;
      }
    }
    else {   // all five robots present
      if (FieldPlayer07_role == "ballL") {
        stdRole = WhiteBoard::STANDARD_ROLE_A;
      }
      else if (FieldPlayer07_role == "ballR") {
        stdRole = WhiteBoard::STANDARD_ROLE_B;
      }
      else if (FieldPlayer07_role == "left") {
        stdRole = WhiteBoard::STANDARD_ROLE_C;
      }
      else if (FieldPlayer07_role == "right") {
        stdRole = WhiteBoard::STANDARD_ROLE_D;
      }
      else {
        stdRole = WhiteBoard::STANDARD_ROLE_E;
      }
    }
    WBOARD->setStandardSituationRole(stdRole);
    
    LOUT << "Player Role: " << FieldPlayer07_role << " StandardSituationRole: " 
         << stdRole << " Spieler: " << WBOARD->getActiveRobots() << endl;
  }
  
  virtual DriveVector getCmd(const Tribots::Time&) 
    throw (Tribots::TribotsException) {
    DriveVector dv; 
    dv.kick = 0;
    dv.vrot = 0;
    dv.vtrans = Vec(0.,0.);
    return dv;
  }


};


class VirtObstacles : public FataMorgana {
  virtual void update(WorldModelTypeBase* wm) throw() {
/*    LOUT << "Virtual obstacles added" << endl;
    wm->add_obstacle_absolute(Vec(0, 1000.), 500.);
    wm->add_obstacle_absolute(Vec(-500, 3000.), 500);
    wm->add_obstacle_absolute(Vec(700, 2000.), 500);
*/
  }
};

class BSupportLongPassConditioned : public BSupportLongPass {
private:
  int counter;
                      
public:
  BSupportLongPassConditioned() : BSupportLongPass("BSupportLongPassConditioned")
  {}
  bool checkCommitmentCondition(const Time& t) throw() {
    if ((WBOARD->getZonePressureRole() != "left" &&
         WBOARD->getZonePressureRole() != "right") ||   // bleibt nur bei links und rechts an
        WBOARD->onlyTwoRobots() ||                      // mindestens 4 roboter, 
        WBOARD->onlyOneRobot() ||                       
        WBOARD->onlyThreeRobots()) {
      return false;
    }

      // if the ball was lost, go only off after an hysteresis
      if (WBOARD->teamPossessBall()){
      counter = 30;
      } else {
         if (counter == 0){
            return false;
         }
         else{
            --counter;
         } 
      }
      return BSupportLongPass::checkCommitmentCondition(t);
  }


  bool checkInvocationCondition(const Time& t) throw() {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    if ((WBOARD->getZonePressureRole() != "left" &&
         WBOARD->getZonePressureRole() != "right") ||   // geht nur bei links und rechts an
        WBOARD->onlyTwoRobots() ||                      // mindestens 4 roboter, 
        WBOARD->onlyOneRobot() ||                      
        WBOARD->onlyThreeRobots()) {
      return false;
    }
    
    LOUT << "check LongPass, teamPosBallExt:  " << 
        WBOARD->teamPossessBallExtended() << endl;
    
    return (WBOARD->teamPossessBallExtended() || WBOARD->teamPossessBall()) &&
      ((WBOARD->getZonePressureRole() == "left" &&      // ball muss deutlich auf der Seite des spielers sein, sonst
        ballLocation.pos.x > +500.) ||                  // wird protect goal oder supportMiddle verwendet
       (WBOARD->getZonePressureRole() == "right" &&
        ballLocation.pos.x < -500.))   &&
      (ballLocation.pos.y > -field.field_length/4.) &&  // nicht mehr vor eigenem Strafraum
      BSupportLongPass::checkInvocationCondition(t);
  }
};




class BSupportNearBallConditioned : public BSupportNearBall {

public:
  BSupportNearBallConditioned() : BSupportNearBall("BSupportNearBallConditioned")
  {}
  bool checkCommitmentCondition(const Time& t) throw() {
    if ((WBOARD->getZonePressureRole() != "ballL" &&
         WBOARD->getZonePressureRole() != "ballR") ||   // bleibt nur bei links und rechts an
        WBOARD->onlyTwoRobots() ||                      // mindestens 5 roboter, 
        WBOARD->onlyOneRobot() ||                       // also safety erforderlich...
        WBOARD->onlyThreeRobots()) {
      return false;
    }
    return
      WBOARD->teamPossessBall() &&
      BSupportNearBall::checkCommitmentCondition(t);
  }
  bool checkInvocationCondition(const Time& t) throw() {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry();
    if ((WBOARD->getZonePressureRole() != "ballL" &&
         WBOARD->getZonePressureRole() != "ballR") ||   // geht nur bei links und rechts an
        WBOARD->onlyTwoRobots() ||                      // mindestens 5 roboter, 
        WBOARD->onlyOneRobot() ||                       // also safety erforderlich...
        WBOARD->onlyThreeRobots()) {
      return false;
    }
    
    if (((WBOARD->getZonePressureRole() == "ballL" && ballLocation.pos.x < 1000.) ||     // Ball ist auf eigener Seite oder im Mittelbereich-> nicht wegfahren, da eventuell noch nicht zwischen ball und tor
         (WBOARD->getZonePressureRole() == "ballR" && ballLocation.pos.x >-1000.)) && 
        ! WBOARD->teamPossessBallExtended() &&
        ballLocation.pos.y < 0) {
      LOUT << "SupportNearBall: nicht angegangen, weil Ball auf eigener Seite und ev. zu gefeaehrlich" << endl;
      return false;
    }
    
    return
      WBOARD->teamPossessBall() && 
      ((WBOARD->getZonePressureRole() == "ballL" &&      
        ballLocation.pos.x > -2000.) ||                   
       (WBOARD->getZonePressureRole() == "ballR" &&
        ballLocation.pos.x < +2000.))   &&
      (ballLocation.pos.y > -field.field_length/2.+field.penalty_area_length) &&  // nicht mehr vor eigenem Strafraum
      BSupportNearBall::checkInvocationCondition(t);
  }
  
  DriveVector getCmd(const Time& t) throw () {
    blockPosition = WBOARD->getZonePressureRole() == "ballL" ? BLOCK_LEFT : BLOCK_RIGHT; // Blockposition erstmal in abhaengigkeit von Rolle setzen
    return BSupportNearBall::getCmd(t);
  }
};



} // Ende unbenannter Namespace

FieldPlayer07::FieldPlayer07 (const ConfigReader& cfg) throw () 
  : BehaviorPlayer ("FieldPlayer07", _tribots_fp07_roles, 5) 
{
//  MWM.add_fata_morgana(new VirtObstacles());
    
  set_role("ballL");  // anfaengliche Rolle setzen
    
  // Das WhiteBoard veranlassen, die Configs auzuwerten ///////////////////////
  WhiteBoard::getTheWhiteBoard()->readConfigs(cfg);

  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  // Area, die der Roboter nicht verlassen darf festlegen (Training) //////////
  // ACHTUNG: Am 23.4.08 auf kleinere Seitenbanden eingestellt, da zu oft in Banden gefahren.
  XYRectangle area(Vec( fgeom.field_width /2. + 0., 
                        fgeom.field_length/2. + 0.),
                   Vec(-fgeom.field_width /2. - 0.,
                       -fgeom.field_length/2. - 0)); 
  
  // Aktivitaetsbereiche fuer Rollen festlegen ////////////////////////////////
  Vec target(0., fgeom.field_length/4.);

  XYRectangle ballLeftActiveArea(Vec(-fgeom.field_width   /2. - 2000., 
                                     fgeom.field_length   /2. + 2000.), 
                                 Vec(fgeom.field_width    /2. -  500.,
                                     -fgeom.field_length  /2. - 2000.));
  XYRectangle ballRightActiveArea(Vec(fgeom.field_width   /2. + 2000., 
                                      fgeom.field_length  /2. + 2000.),
                                  Vec(-fgeom.field_width  /2. +  500.,
                                      -fgeom.field_length /2. - 2000.));
  
  // von gegn. grundlinie bis 2000.mm hinter grundlinie, von 2000.mm ausserhalb
  // der Seitenlinie bis 750.mm ber die tor-tor-linie (1,5m ueberlappung)
  XYRectangle leftActiveArea(Vec(-fgeom.field_width/2. - 2000., 
                                 fgeom.field_length/2.), 
                             Vec(3250., 
                                 -fgeom.field_length/2.-2000.));
  XYRectangle rightActiveArea(Vec(fgeom.field_width/2. + 2000., 
                                  fgeom.field_length/2),
                              Vec(-3250., 
                                  -fgeom.field_length/2.-2000.));
  // Bis etwas vor dem Torraum, nicht bis ganz zur seitenlinie
  XYRectangle safetyActiveArea(Vec(-fgeom.penalty_area_width/2.-1500., 
                                   -fgeom.field_length/2.-1000.),  // nicht sonderlich weit rausfahren
                               Vec(+fgeom.penalty_area_width/2.+1500., 
                                   -fgeom.field_length/2.+fgeom.penalty_area_length+2000.));

  // Kickdauern und Passwahrscheinlichkeiten einlesen /////////////////////////
  int longPassKickDuration = 40;
  int shortPassKickDuration = 20;
  int throwInKickDuration = 7; // nur fuer roboter mit harting kicker relevant
  int shotKickDuration = 34; // nur fuer roboter mit harting kicker relevant // GESCHWINDIGKEITSHACK
  double standardPassProbability = 0.75;
  double standardDribbling = 0.25;
  int standardPositioning = 0;
  
  if (cfg.get(("FieldPlayer07::shortPassKickDuration"), shortPassKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer07::shortPassKickDuration");
  }    
  if (cfg.get(("FieldPlayer07::longPassKickDuration"), longPassKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer07::longPassKickDuration");
  }   
  if (cfg.get(("FieldPlayer07::throwInKickDuration"), throwInKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer07::throwInKickDuration");
  } 
  if (cfg.get(("FieldPlayer07::shotKickDuration"), shotKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer07::shotKickDuration");
  } 
  if (cfg.get(("FieldPlayer07::standardPassProbability"), standardPassProbability) < 0) {
    throw InvalidConfigurationException("FieldPlayer07:standardPassProbability");
  }   
  if (cfg.get(("FieldPlayer07::standardDribbling"), standardDribbling) < 0) {
    throw InvalidConfigurationException("FieldPlayer07:standardDribbling");
  }   
  if (cfg.get(("FieldPlayer07::standardPositioning"), standardPositioning) < 0) {
    throw InvalidConfigurationException("FieldPlayer07:standardPositioning");
  } 

  // Optionsstack fuellen /////////////////////////////////////////////////////
  
  addOption (new BGameStopped());

 addOption (new BTestStateStop());   // Was ist das Stefan
  addOption (new BLeaveGoal());

 // addOption (new BOwnStandardSituation());  // von lucas ?

  addOption (new BPreOwnIndirectStandardSituation(
                                                          shortPassKickDuration,
                                                          longPassKickDuration,throwInKickDuration,
                                                          standardPassProbability,
                                                          standardDribbling, standardPositioning)); 
  
  addOption (new BOwnPenalty());
  addOption (new BStuckStandardSituationFP07());
  addOption (new BVolleyApproachAfterOwnSetPlay());
  addOption (new BPostOpponentStandardSituation());
  addOption (new BPreOpponentStandardSituationNew(false));
  addOption (new BStayInsideArea(area, target)); // Training
  //addOption (new BPreventInterferenceWithPassConditioned());
  addOption (new BCounterAttackConditioned());
  addOption (new BallOwnerStack(ballLeftActiveArea,   // Active areas define areas, where
                                        ballRightActiveArea,  // the robot potentially will go towards the ball.
                                        leftActiveArea,       // outside these areas the robot may only go
                                        rightActiveArea,      // towards the ball in case of a "help" 
                                        safetyActiveArea,
                                        longPassKickDuration,
                                        shotKickDuration));   // call by a teammate
  addOption (new BSupportLongPassConditioned());  // left supports ball owning ballR (and vice versa) at a long pass position
  addOption (new BSupportNearBallConditioned());  // ballX Supports ball owning ballY at a pos slightly behind the ball towards the center
  addOption (new ZonePressure());
  addOption (new BPatrolFP07());
  addOption (new BEmergencyStop());
  addOption (new BFP07Update());
}

FieldPlayer07::~FieldPlayer07 () throw () 
{}
