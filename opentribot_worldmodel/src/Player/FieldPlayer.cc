#include "FieldPlayer.h"
#include "PlayerFactory.h"
#include "WhiteBoard.h"
#include "../Fundamental/geometry.h"
#include "../WorldModel/WorldModel.h"
#include "../WorldModel/FataMorgana.h"
#include "../Behavior/Skills/WithoutBall/SPatrol.h"
#include "../Behavior/Behaviors/ApproachingBall/BComplexApproachBallFreePlay.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BWingAttack.h"
#include "../Behavior/Behaviors/BallHandling/BPassSpontaneously.h"
#include "../Behavior/Behaviors/BallHandling/BBefreiungsschlag.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToPassPosition.h"
#include "../Behavior/Behaviors/BallHandling/BEigenMove.h"
#include "../Behavior/Behaviors/BallHandling/BStuckOwnsBall.h"
#include "../Behavior/Behaviors/BallHandling/BStuckDistanceShooter.h"
#include "../Behavior/Behaviors/BallHandling/BPassSpontaneously.h"
#include "../Behavior/Behaviors/BasicMovements/BShootEmergency.h"
#include "../Behavior/Behaviors/BasicMovements/BShoot.h"
#include "../Behavior/Behaviors/BasicMovements/BDraufhalten.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/WithoutBall/BBlockWayToGoal.h"
#include "../Behavior/Behaviors/WithoutBall/BBlockWayToMiddle.h"
#include "../Behavior/Behaviors/WithoutBall/BBlockWayToOpponentGoal.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPostOpponentStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOwnIndirectStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BOwnPenalty.h"
#include "../Behavior/Behaviors/SpecialGameStates/BTestBehavior.h"
 
// Innerhalb dieser Datei wird die spielertypspezifische Strategie des 
// Feldspielers festgelegt. Die vom Feldspieler verwendeten Behaviors und
// Skills sollten soweit wie mï¿½lich generisch gehalten werden, d.h. 
// Strategiespezifische Einstellungen wie Aktionsbereiche und 
// Rollenspezifikationen sollten aussschlieï¿½ich hier ber das setzen von 
// allgemeinen Parametern der Behaviors und durch Ableiten und ï¿½erschreiben 
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
        string("Feldspieler"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, 
			    PlayerType*) 
      throw (TribotsException,std::bad_alloc) 
    {
      return new FieldPlayer (reader);
    }
  };
  Builder the_builder;
}


namespace Tribots{   // unbenannter Namespace notwenig, um Verwechslungen beim Linken mit FieldPlayer zu vermeiden


static string fieldplayer_role = "ball";


//
// BPatrol verwendet den Skill SPatrol mit geeigneten Patrolpositionen,
// die von der aktuellen Rolle abgeleitet werden.
//
class BPatrolFP : public Behavior {
public:
  
  BPatrolFP() : Behavior("BPatrolFP"), patrol(new SPatrol()) {};
  ~BPatrolFP() throw() { delete patrol; }
  
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);
    return (ball.pos_known == BallLocation::unknown);
  }
  virtual bool checkInvocationCondition(const Time& t) throw() {
    return BPatrolFP::checkCommitmentCondition(t);
  }
  virtual DriveVector getCmd(const Time& t) throw(TribotsException) {
    const FieldGeometry& field = MWM.get_field_geometry();      
      
    if (fieldplayer_role == "ball")  {
        std::vector<Vec> positions;
        positions.push_back(Vec(-2*field.center_circle_radius, field.center_circle_radius));
        positions.push_back(Vec(-field.center_circle_radius, 2*field.center_circle_radius));
        positions.push_back(Vec(field.center_circle_radius, 2*field.center_circle_radius));
        positions.push_back(Vec(2*field.center_circle_radius, field.center_circle_radius));
        patrol->setPatrolPositions(Vec(0,0), Vec(0, 0)); // wegen dynamischem Rollenwechsel nötig TODO: remove this constructor
        patrol->setPatrolPositions(positions);
    }
    else if (fieldplayer_role == "left") {
      patrol->setPatrolPositions(Vec(-field.penalty_area_width/2,
                                     -field.field_length/2+field.penalty_area_length),
                                 Vec(-500, -field.field_length/2+field.penalty_area_length+500));
    }
    else {
      patrol->setPatrolPositions(Vec(field.penalty_area_width/2,
                                     -field.field_length/2+field.penalty_area_length),
                                 Vec(500, -field.field_length/2+field.penalty_area_length + 500));
    }
    return patrol->getCmd(t);
  }
  
protected:
  SPatrol* patrol;
};

//
// BStuckOwnsBall abgeleitet fuer Verwendung in Feldspieler 
//
class BStuckOwnsBallConditioned : public BStuckOwnsBall {
public:
  BStuckOwnsBallConditioned() : BStuckOwnsBall(), act(true) 
  { name = "BStuckOwnsBallConditioned"; }
  virtual bool checkInvocationCondition(const Time& t) throw() {
    const RobotLocation& robot = MWM.get_robot_location(t);
    return 
      act && BStuckOwnsBall::checkInvocationCondition(t) && 
      (robot.pos.y > 500. ||
       MWM.get_game_state().refstate == preOwnThrowIn ||
       MWM.get_game_state().refstate == preOwnGoalKick ||
       MWM.get_game_state().refstate == preOwnCornerKick ||
       MWM.get_game_state().refstate == preOwnFreeKick ||
       MWM.get_game_state().refstate == preOwnKickOff);
   }
   void updateTactics (const TacticsBoard& tb) throw () {
     if (tb[string("Stuck")]==string("wegfahren"))
       act=true;
     else if (tb[string("Stuck")]==string("drehen"))
       act=false;
   }
protected:
  bool act;
};


//
// BStuckStandardSituation
//
class BStuckStandardSituation : public BStuckOwnsBall {
public:
  BStuckStandardSituation() : BStuckOwnsBall(), act(true) 
  { name = "BStuckStandardSituation"; }
  virtual bool checkInvocationCondition(const Time& t) throw() {
//    const RobotLocation& robot = MWM.get_robot_location(t);
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

//
// Ballstack - geht immer an, wenn der Spieler zum Ball darf
//
class BallStack : public BDIBehavior {
private:
  Area* leftActionArea; ///< Aktivitï¿½sbereich des linken Verteidigers
  Area* rightActionArea;///< Aktivitï¿½sbereich des rechten Verteidigers
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
    
    if ((WBOARD->teamPossessBall() || WBOARD->teammateNearBall()) && 
        ! _receivePass) {
      return false;
    }
    if (ball.pos_known == BallLocation::unknown || 
        ball.pos_known == BallLocation::raised) {
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
  BallStack(const Area& leftActionArea,
            const Area& rightActionArea,
	    bool obeyPenaltyAreaRules=true) 
    : BDIBehavior("FieldPlayerBallStack"), 
      leftActionArea(leftActionArea.clone()), 
      rightActionArea(rightActionArea.clone()), counter(0),
      obeyPenaltyAreaRules(obeyPenaltyAreaRules)
  {
    //addOption (new BTestBehavior());
    
    addOption (new BShootEmergency());
    addOption (new BDraufhalten(.25));
    addOption (new BShoot());
    addOption (new BBefreiungsschlag());
    addOption (new BStuckOwnsBallConditioned());
    addOption (new BPassSpontaneously());
    addOption (new BStuckDistanceShooter());
    addOption (new BEigenMove());
    addOption (new BWingAttack(2.5, 1., true));
    addOption (new BDribbleBallStraightToGoalEvadeSidewards());
    addOption (new BDribbleBallToGoal());
    addOption (new BComplexApproachBallFreePlay());
    lastRoleRequest.update();   
  }
  
  ~BallStack() throw () {
    delete leftActionArea;
    delete rightActionArea;
  }

  /**
   * Generiert einen DriveVector. Verwendet dazu die unveraenderte Methode
   * des BDI-Verhaltens (siehe BDIBehavior), ueberprueft aber vorher, ob
   * beim Teamcontrol die Rolle "ball" aktiv beantragt werden soll.
   */
  virtual DriveVector getCmd(const Tribots::Time& t)     
    throw (Tribots::TribotsException) { 
    // Anfrage an das teamcontrol, Rolle "ball" zu bekommen.
    if (fieldplayer_role != "ball" && WBOARD->doPossessBall(t) 
	&& t.diff_sec(lastRoleRequest) > 1) { // nicht zu haeufig anfragen 
      MWM.get_message_board().publish("request_role_ball");
      LOUT << "request_role_ball send" << endl;
      lastRoleRequest = t;
    }
    return BDIBehavior::getCmd(t);
  }

  virtual void cycleCallBack(const Time& t) throw() {
    BDIBehavior::cycleCallBack(t);
    WBOARD->checkMessageBoard();
    if (WBOARD->receivePass()) {
      LOUT << "in cycleCallBack of BallStack: heard 'receive_pass'" << endl;
      _lastPassTime = t;
      _lastPassTime.add_sec(5);
      _receivePass = true;
    }
    if (_lastPassTime < t) {
      _receivePass = false;
    }
  }

  virtual bool checkCommitmentCondition(const Time& t) throw() {
    if (!checkConditions(t)) { // auf jeden fall aufhï¿½en, wenn nicht erfllt
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
    if ((fieldplayer_role != "ball" &&  // zum ball nur, wenn ballspieler
         !((fieldplayer_role == "left" && // oder linker spieler und 
            leftActionArea->is_inside(ball.pos.toVec()))) && // ball ist links
         !((fieldplayer_role == "right" && // oder rechter spieler und
            rightActionArea->is_inside(ball.pos.toVec()))) // ball ist rechts
         ) && counter == 0 && 
        !(fieldplayer_role == "left" && WBOARD->onlyTwoRobots() &&
          (leftActionArea->is_inside(ball.pos.toVec()) ||
           rightActionArea->is_inside(ball.pos.toVec())))) {  
                               // 30 zyklen nach ballverlust auch ausserhalb
                               // des 
      return false;            // zustï¿½digkeitsbereichs fahren
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
    if (obeyPenaltyAreaRules && fieldplayer_role != "ball" && 
        penaltyArea.is_inside(ball.pos.toVec())) { // nicht an, wenn der Ball im Strafraum
      return false;
    }
    return true;   // im zustï¿½digkeitsbereich, oder in den letzten 20 zyklen
  }                // mal den ball gehabt

  virtual bool checkInvocationCondition(const Time& t) throw() {
    if (!checkConditions(t)) {
      return false;
    }
    LOUT << "After checkConditions" << endl;
    const BallLocation& ball = MWM.get_ball_location(t);
    if (fieldplayer_role != "ball" &&  // zum ball nur, wenn ballspieler
        !((fieldplayer_role == "left" && // oder linker spieler und 
           leftActionArea->is_inside(ball.pos.toVec()))) && // ball ist links
        !((fieldplayer_role == "right" && // oder rechter spieler und
           rightActionArea->is_inside(ball.pos.toVec())) && 
          !(fieldplayer_role == "left" && WBOARD->onlyTwoRobots() &&
            (leftActionArea->is_inside(ball.pos.toVec()) ||
             rightActionArea->is_inside(ball.pos.toVec())))) && // ball ist rechts
        !WBOARD->doPossessBall(t)
        ) {
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
    if (!WBOARD->doPossessBall(t) &&
        obeyPenaltyAreaRules && fieldplayer_role != "ball" && 
        penaltyArea.is_inside(ball.pos.toVec())) { // nicht an, wenn der Ball im Strafraum
      return false;
    }
    return true;
  }  

  virtual void loseControl(const Time&) throw(TribotsException) 
  { counter = 0; }
};   

class BBlockWayToMiddleConditioned : public BBlockWayToMiddle {
private:
  virtual bool checkConditions(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);
    return 
      (fieldplayer_role == "left" && ball.pos.x > 750. ||
       fieldplayer_role == "right" && ball.pos.x < -750.) &&
      ! (fieldplayer_role == "left" && WBOARD->onlyTwoRobots()); // da muss 
         //  block goal angehen, weil rechter spieler fehlt    
   }
public:
  BBlockWayToMiddleConditioned() : BBlockWayToMiddle() { name = "FP::BBlockWayToMiddleConditioned"; }
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    return 
      BBlockWayToMiddle::checkCommitmentCondition(t) && checkConditions(t);
  }
  
  virtual bool checkInvocationCondition(const Time& t) throw() {
    return 
      BBlockWayToMiddle::checkInvocationCondition(t) && checkConditions(t);
  }
};

class BBlockWayToOpponentGoalConditioned : public BBlockWayToOpponentGoal {
private:
  virtual bool checkConditions(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);
    return 
      (fieldplayer_role == "left" && ball.pos.x > 750. ||
       fieldplayer_role == "right" && ball.pos.x < -750.) &&
      ! (fieldplayer_role == "left" && WBOARD->onlyTwoRobots()); // da muss 
         //  block goal angehen, weil rechter spieler fehlt    
   }
public:
  BBlockWayToOpponentGoalConditioned() : BBlockWayToOpponentGoal() { name = "FP::BBlockWayToOpponentGoalConditioned"; }
  virtual bool checkCommitmentCondition(const Time& t) throw() {
    return 
      BBlockWayToOpponentGoal::checkCommitmentCondition(t) && checkConditions(t);
  }
  
  virtual bool checkInvocationCondition(const Time& t) throw() {
    return 
      BBlockWayToOpponentGoal::checkInvocationCondition(t) && checkConditions(t);
  }
};
/** ball links:  LEFT: blockGoal, RIGHT: blockMiddle,
 *  ball mitte:  LEFT, RIGHT: beide blockGoal
 *  ball rechts: LEFT: blockMiddle, RIGHT: blockGoal
 */
class DefendStack : public BDIBehavior
{
private:
  virtual bool checkConditions(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);

    if (ball.pos_known == BallLocation::unknown ||
        ball.pos_known == BallLocation::raised) {
      return false;
    }
    return true;
  }
public:
  DefendStack() : BDIBehavior("FieldPlayerDefendStack")
  {
    const FieldGeometry& field = MWM.get_field_geometry(); 

    //addOption (new BBlockWayToMiddleConditioned());
    addOption (new BBlockWayToOpponentGoalConditioned());
    addOption (new BBlockWayToGoal(field.field_length));
  }

  virtual bool checkCommitmentCondition(const Time& t) throw() {
    return checkConditions(t);
  }

  virtual bool checkInvocationCondition(const Time& t) throw() {
    return checkConditions(t);
  }
};

static const char *_tribots_fp_roles[3] = { "ball", "left", "right" };

bool
FieldPlayer::set_role(const char* role) throw ()
{
  if (BehaviorPlayer::set_role(role)) {
    fieldplayer_role = std::string(role);      // fuer "interne" behaviors
    return true;
  }
  return false;
}

class BStandardSituationRoleUpdate : public Behavior {
public:
  BStandardSituationRoleUpdate() : Behavior("BStandardSituationRoleUpdate")
  {}

  virtual bool checkInvocationCondition(const Time& t) throw() {
    return false;
  }
  
  virtual void cycleCallBack(const Time&) throw() {
    // nun noch die rolle in standardsituationen setzen
    int stdRole = WhiteBoard::STANDARD_ROLE_AB;
    if (fieldplayer_role == "ball" &&
        WBOARD->onlyOneRobot()) {
      stdRole = WhiteBoard::STANDARD_ROLE_ABCDE;
    }
    else if (fieldplayer_role == "left") {
      if (WBOARD->onlyTwoRobots()) {
        stdRole = WhiteBoard::STANDARD_ROLE_CDE;
      }
      else {
        stdRole = WhiteBoard::STANDARD_ROLE_C;
      }
    }
    else if (fieldplayer_role == "right") {
      if (WBOARD->onlyTwoRobots()) {
        stdRole = WhiteBoard::STANDARD_ROLE_CDE;
      }
      else {
        stdRole = WhiteBoard::STANDARD_ROLE_DE;
      }
    }
    else if (fieldplayer_role == "middle") {
      stdRole = WhiteBoard::STANDARD_ROLE_CDE;
    }
    WBOARD->setStandardSituationRole(stdRole);
    
    LOUT << "Role: " << fieldplayer_role << " StandardSituationRole: " 
         << stdRole << endl;
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
    LOUT << "Virtual obstacles added" << endl;
    wm->add_obstacle_absolute(Vec(0, 1000.), 500.);
    wm->add_obstacle_absolute(Vec(-500, 3000.), 500);
    wm->add_obstacle_absolute(Vec(700, 2000.), 500);
  }
};

    
} // Ende unbenannter Namespace



FieldPlayer::FieldPlayer (const ConfigReader& cfg) throw () 
  : BehaviorPlayer ("FieldPlayer", _tribots_fp_roles, 3) 
{
//  MWM.add_fata_morgana(new VirtObstacles());
    
  // Das WhiteBoard veranlassen, die Configs auzuwerten ///////////////////////
  WhiteBoard::getTheWhiteBoard()->readConfigs(cfg);

  const FieldGeometry& fgeom = MWM.get_field_geometry();
  
  // Area, die der Roboter nicht verlassen darf festlegen (Training) //////////
  XYRectangle area(Vec( fgeom.field_width/2.  + 500., 
                        fgeom.field_length/2. + 500.),
                   Vec(-fgeom.field_width/2.  - 500.,
                       -fgeom.field_length/2. - 500)); 
  
  // Aktivitï¿½sbereiche festlegen /////////////////////////////////////////////
  Vec target(0., fgeom.field_length/4.);
  // von mittellinie bis 2000.mm hinter grundlinie, von 2000.mm auï¿½rhalb
  // der Seitenlinie bis 750.mm ber die tor-tor-linie (1,5m berlappung)
  XYRectangle leftActiveArea(Vec(-fgeom.field_width/2. - 2000., 
                                 fgeom.field_length/2), 
                             Vec(750., -fgeom.field_length/2.-2000.));
  XYRectangle rightActiveArea(Vec(fgeom.field_width/2. + 2000., 
                                  fgeom.field_length/2),
                              Vec(-750., -fgeom.field_length/2.-2000.));

  // Kickdauern und Passwahrscheinlichkeiten einlesen /////////////////////////
  int longPassKickDuration = 40;
  int shortPassKickDuration = 20;
  double standardPassProbability = 0.75;
  double standardDribbling = 0.25;
  int standardPositioning = 0;
  
  if (cfg.get(("FieldPlayer::shortPassKickDuration"), shortPassKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer::shortPassKickDuration");
  }    
  if (cfg.get(("FieldPlayer::longPassKickDuration"), longPassKickDuration) < 0) {
    throw InvalidConfigurationException("FieldPlayer::longPassKickDuration");
  }   
  if (cfg.get(("FieldPlayer::standardPassProbability"), standardPassProbability) < 0) {
    throw InvalidConfigurationException("FieldPlayer:standardPassProbability");
  }   
  if (cfg.get(("FieldPlayer::standardDribbling"), standardDribbling) < 0) {
    throw InvalidConfigurationException("FieldPlayer:standardDribbling");
  }   
  if (cfg.get(("FieldPlayer::standardPositioning"), standardPositioning) < 0) {
    throw InvalidConfigurationException("FieldPlayer:standardPositioning");
  } 

  // Optionsstack fuellen /////////////////////////////////////////////////////
  
/*  addOption (new BPreOwnIndirectStandardSituation(
    shortPassKickDuration,
    longPassKickDuration,standardPassProbability,
    standardDribbling, standardPositioning)); */
  
  addOption (new BPreOwnIndirectStandardSituation(
                                                          shortPassKickDuration,
                                                          longPassKickDuration,7,standardPassProbability,
                                                          standardDribbling, standardPositioning));
  
  addOption (new BOwnPenalty());
  addOption (new BStuckStandardSituation());
  addOption (new BPostOpponentStandardSituation());
  addOption (new BPreOpponentStandardSituation(false));
  addOption (new BGameStopped());
  addOption (new BStayInsideArea(area, target)); // Training
  addOption (new BallStack(leftActiveArea, rightActiveArea));
  addOption (new DefendStack());
  addOption (new BPatrolFP());
  addOption (new BEmergencyStop());
  addOption (new BStandardSituationRoleUpdate());
}

FieldPlayer::~FieldPlayer () throw () 
{}
