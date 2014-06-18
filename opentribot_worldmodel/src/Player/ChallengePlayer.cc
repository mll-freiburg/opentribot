#include "ChallengePlayer.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"

#include "WhiteBoard.h"

#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/BallHandling/BPassSpontaneously.h"
#include "../Behavior/Behaviors/ApproachingBall/BComplexApproachBallFreePlay.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToGoal.h"
#include "../Behavior/Behaviors/BallHandling/BWingAttack.h"
#include "../Behavior/Behaviors/BallHandling/BPassChallenge.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToPassPosition.h"
#include "../Behavior/Behaviors/BallHandling/BEigenMove.h"
#include "../Behavior/Behaviors/BallHandling/BStuckOwnsBall.h"
#include "../Behavior/Behaviors/BallHandling/BStuckDistanceShooter.h"
#include "../Behavior/Behaviors/BasicMovements/BShootEmergency.h"
#include "../Behavior/Behaviors/BasicMovements/BShoot.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/WithoutBall/BBlockWayToGoal.h"
#include "../Behavior/Behaviors/WithoutBall/BBlockWayToMiddle.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("ChallengePlayer"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,std::bad_alloc) {
      return new ChallengePlayer (reader);
    }
  };
  Builder the_builder;
}

static string fieldplayer_role = "shooter";
static float changeLine=0.;

// -------------------------------------------------------------------------------------------------------------------------------

class BGoBack : public Behavior
  {
  public:
    SGoToPosEvadeObstacles* skill;;
  
  BGoBack() 
    : Behavior("BGoBack"), skill(new SGoToPosEvadeObstacles())
  {}

  ~BGoBack() throw ()
  {
    delete skill;
  }

  DriveVector
  getCmd(const Time& t) throw(TribotsException)
  {
    const FieldGeometry& field = MWM.get_field_geometry(); 

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec guestGoalPos(0., -1000+field.field_length / 2.);
    Vec ownGoalPos(0., +1000-field.field_length / 2.);
 
    Vec pongLocation=ownGoalPos;
    
    skill->init(pongLocation, 2.0, guestGoalPos);
    return skill->getCmd(t);  
  }

  bool 
  checkInvocationCondition(const Time& tt) throw() {
    const BallLocation& ballLocation = MWM.get_ball_location(tt);
    LOUT<<"ballpos.y: "<<ballLocation.pos.toVec().y<<endl;
    if ( (ballLocation.pos.toVec().y>changeLine)&&(fieldplayer_role == "shooter") )
      return true;
  
    //bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    //bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
    // return seheBall || seheKommuniziertenBall;
    return false;
  }

  bool
  checkCommitmentCondition(const Time& tt) throw()
  {
    return checkInvocationCondition(tt);
  }
};


// ------------------------------------------------------------------------------------------------------------------------------

class BReceivePass : public Behavior
  {
  public:
    SGoToPosEvadeObstacles* skill;;
    PIDController headingController;
    double maxDistance;
    bool obeyPenaltyAreaRules;
    bool receivedPass;
  
  BReceivePass(double maxDistance, bool obeyPenaltyAreaRules) 
    : Behavior("BReceivePass"), skill(new SGoToPosEvadeObstacles()),
      headingController(4., 0.0000 , 0.00, 3., -3.), maxDistance(maxDistance),
      obeyPenaltyAreaRules(obeyPenaltyAreaRules)
  {
    receivedPass = 0;
  }

  ~BReceivePass() throw ()
  {
    delete skill;
  }

  DriveVector
  getCmd(const Time& t) throw(TribotsException)
  {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    const FieldGeometry& field = MWM.get_field_geometry(); 
    const RobotLocation& robot = MWM.get_robot_location(t);

    Frame2d world2robot = WBOARD->getAbs2RelFrame(t);

    Vec guestGoalPos(0., +300+field.field_length / 2.);
    Vec pongLocation=guestGoalPos+(ballLocation.pos.toVec()-guestGoalPos)*0.75;
  
    // Der ball-spieler darf in die area rein, waehrend die beiden 
    // pong spieler draussen bleiben muessen.
    XYRectangle penaltyArea(Vec(-field.penalty_area_width/2. - 200, 
                                -field.field_length / 2. - 800), // hinter gl
                            Vec(field.penalty_area_width/2. + 200,
                                -field.field_length / 2. + 200 +
                                field.penalty_area_length));
    
    LineSegment leftLineOfPenaltyArea(Vec(-field.penalty_area_width/2.-200, 
                                          -field.field_length / 2. - 200.),
                                      Vec(-field.penalty_area_width/2.-200, 
                                          -field.field_length / 2. + 
                                          field.penalty_area_length+100));
    LineSegment rightLineOfPenaltyArea(Vec(field.penalty_area_width/2.+200, 
                                           -field.field_length / 2. - 200.),
                                       Vec(field.penalty_area_width/2.+200, 
                                           -field.field_length / 2. + 
                                           field.penalty_area_length+100));
    LineSegment topLineOfPenaltyArea(Vec(-field.penalty_area_width/2.-200, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length+100),
                                     Vec(field.penalty_area_width/2.+200, 
                                         -field.field_length / 2. + 
                                         field.penalty_area_length+100));
    
    if (penaltyArea.is_inside(ballLocation.pos.toVec()) &&
        penaltyArea.is_inside(pongLocation) && obeyPenaltyAreaRules) {     // Ball in Strafraum
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
    }
    else if (penaltyArea.is_inside(pongLocation)) {// Roboter soll in Strafraum
      // sucht den Punkt an der Strafraumgrenze, der auf der abzudeckenden Linie
      // Ball->Tor liegt.
      LineSegment ballLine(ballLocation.pos.toVec(), guestGoalPos);
      
      vector<Vec> intersections;
      intersections = intersect(leftLineOfPenaltyArea, ballLine);
      if (! intersections.size()) {
        intersections = intersect(rightLineOfPenaltyArea, ballLine);
      }
      if (! intersections.size()) {
        intersections = intersect(topLineOfPenaltyArea, ballLine);
      }
      if (intersections.size() > 1) {
        pongLocation = intersections[0];
      }
    }
    
    if (((pongLocation-guestGoalPos).length()) > maxDistance) {
      pongLocation = guestGoalPos + 
        (ballLocation.pos.toVec()-guestGoalPos).normalize() * maxDistance; 
    }
    
    // check, if position is free, choose position behind and to middle, 
    // if not.
    bool posFree = true;
    const ObstacleLocation obstacles = MWM.get_obstacle_location(t);
    for (unsigned int i=0; i < obstacles.size(); i++) {
      if ((obstacles[i].pos-pongLocation).length() < 600.) {
        posFree = false;
        break;
      }
    }

    if (!posFree) {
      if (ballLocation.pos.x > 200.) {
        pongLocation += Vec(-600., -700);  // links hinter dem anderen
      }
      else if (ballLocation.pos.x < -200.) {
        pongLocation += Vec(600., -700);   //rechts hinter dem anderen
      }
      else { 
        if (robot.pos.x > 0) {             // da bleiben, wo der roboter ist
          pongLocation += Vec(600., -700.);
        }
        else {
          pongLocation += Vec(-600., -700.);
        }
      }
    }

    skill->init(pongLocation, 2.5, ballLocation.pos.toVec()-guestGoalPos);
    return skill->getCmd(t);  
  }

  bool 
  checkInvocationCondition(const Time& tt) throw() {
    const BallLocation& ballLocation = MWM.get_ball_location(tt);
    LOUT<<"ballpos.y: "<<ballLocation.pos.toVec().y<<endl;
    if ( (ballLocation.pos.toVec().y<changeLine)&&!receivedPass )
      return true;
  
    //bool seheBall=MWM.get_ball_location(tt).pos_known==BallLocation::known;
    //bool seheKommuniziertenBall=MWM.get_ball_location(tt).pos_known==BallLocation::communicated;    
    // return seheBall || seheKommuniziertenBall;
    return false;
  }

  bool
  checkCommitmentCondition(const Time& tt) throw()
  {
    return checkInvocationCondition(tt);
  }
  
  virtual void cycleCallBack(const Time& t) throw() {
    if (MWM.get_game_state().refstate == stopRobot) receivedPass=0;
    LOUT << "received Pass = " << receivedPass << endl;
  }
    
  virtual void loseControl(const Time&) throw(TribotsException) 
  { receivedPass = 1; }
};


// ------------------------------------------------------------------------------------------------------------------------------

class BShooterStack : public BDIBehavior {
private:
  Area* leftActionArea; ///< Aktivitätsbereich des linken Verteidigers
  Area* rightActionArea;///< Aktivitätsbereich des rechten Verteidigers
  int counter;          ///< zur Ueberbrueckung weniger Zyklen ohne Ballbesitz
  Time _lastPassTime;   ///< Wann wurde die letzte Passentscheidung getroffen?
  bool _receivePass;    ///< Soll dieser Spieler gerade einen Pass annehmen?
  Time lastRoleRequest; ///< Wann wurde zuletzt eine neue Rolle erbeten?
  bool obeyPenaltyAreaRules;///< An die nur-1-Spieler-im-Strafraum Regel halten?
  bool playedPass;

  /*
  virtual bool checkConditions(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);
    WBOARD->checkMessageBoard();  // brauche Infos aus dem Messageboard
    
    if (ball.pos_known == BallLocation::unknown || 
        ball.pos_known == BallLocation::raised) {
      return false;
    }
       
    return true;
  }
  */
  
public:

  BShooterStack() 
    : BDIBehavior("ChallengePlayerShooterStack")
  {   
    playedPass=0;
    addOption (new BShootEmergency());
    addOption (new BShoot());
    addOption (new BPassChallenge());
    addOption (new BEigenMove());
    addOption (new BDribbleBallStraightToGoalEvadeSidewards());
    addOption (new BDribbleBallToGoal());
    addOption (new BComplexApproachBallFreePlay());
    lastRoleRequest.update();   
  }
  
  ~BShooterStack() throw () {
  }

  virtual DriveVector getCmd(const Tribots::Time& t)     
    throw (Tribots::TribotsException) { 
    // Anfrage an das teamcontrol, Rolle "shooter" zu bekommen.
    /*
    if (fieldplayer_role != "shooter" && WBOARD->doPossessBall(t) 
	&& t.diff_sec(lastRoleRequest) > 1) { // nicht zu haeufig anfragen 
      MWM.get_message_board().publish("request_role_shooter");
      LOUT << "request_role_shooter send" << endl;
      lastRoleRequest = t;
    }
    */
    return BDIBehavior::getCmd(t);
  }
  /*
  virtual void cycleCallBack(const Time& t) throw() {
    BDIBehavior::cycleCallBack (t);
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
  */

  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const BallLocation& ballLocation = MWM.get_ball_location(t);
    LOUT<<"ballpos.y: "<<ballLocation.pos.toVec().y<<endl;
    if ( (ballLocation.pos.toVec().y<changeLine)&&(fieldplayer_role == "shooter")&&!playedPass )
    //if (fieldplayer_role == "shooter")
      return true;
    /*
    if (!checkConditions(t)) { // auf jeden fall aufhören, wenn nicht erfüllt
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
    */
    /*
    const BallLocation& ball = MWM.get_ball_location(t);
    if ((fieldplayer_role != "shooter" &&  // zum ball nur, wenn ballspieler
         !((fieldplayer_role == "receiver" && // oder linker spieler und 
            leftActionArea->is_inside(ball.pos.toVec()))) && // ball ist links
         !((fieldplayer_role == "right" && // oder rechter spieler und
            rightActionArea->is_inside(ball.pos.toVec()))) // ball ist rechts
         ) && counter == 0 && 
        !(fieldplayer_role == "left" && WBOARD->onlyTwoRobots() &&
          (leftActionArea->is_inside(ball.pos.toVec()) ||
           rightActionArea->is_inside(ball.pos.toVec())))) {  
                               // 30 zyklen nach ballverlust auch ausserhalb
                               // des 
      return false;            // zuständigkeitsbereichs fahren
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
    */
    return false;   // im zuständigkeitsbereich, oder in den letzten 20 zyklen
  }                // mal den ball gehabt

  virtual bool checkInvocationCondition(const Time& t) throw() {
    return checkCommitmentCondition(t);
    /*
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
	    LOUT << "Kein Ballbesitz " << WBOARD->doPossessBall(t) << endl;
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
    */
    return true;
  }  
  
  virtual void cycleCallBack(const Time& t) throw() {
    BDIBehavior::cycleCallBack (t);
    if (MWM.get_game_state().refstate == stopRobot) playedPass=0;;
    LOUT << "played Pass = " << playedPass << endl;
  }

  virtual void loseControl(const Time&) throw(TribotsException) 
  {
    counter = 0;
    playedPass = 1;
  }
};   

// ------------------------------------------------------------------------------------------------------------------------------

class BReceiverStack : public BDIBehavior {
private:
  Area* leftActionArea; ///< Aktivitätsbereich des linken Verteidigers
  Area* rightActionArea;///< Aktivitätsbereich des rechten Verteidigers
  int counter;          ///< zur Ueberbrueckung weniger Zyklen ohne Ballbesitz
  Time _lastPassTime;   ///< Wann wurde die letzte Passentscheidung getroffen?
  bool _receivePass;    ///< Soll dieser Spieler gerade einen Pass annehmen?
  Time lastRoleRequest; ///< Wann wurde zuletzt eine neue Rolle erbeten?
  bool obeyPenaltyAreaRules;///< An die nur-1-Spieler-im-Strafraum Regel halten?

public:
  BReceiverStack() 
    : BDIBehavior("ChallengePlayerReceiverStack")
  {   
    addOption (new BShootEmergency());
    addOption (new BShoot());
    addOption (new BPassSpontaneously());
    addOption (new BEigenMove());
    addOption (new BDribbleBallStraightToGoalEvadeSidewards());
    addOption (new BDribbleBallToGoal());
    addOption (new BReceivePass(4000,0));
    addOption (new BComplexApproachBallFreePlay());
    lastRoleRequest.update();   
  }
  
  ~BReceiverStack() throw () {
  }

  virtual DriveVector getCmd(const Tribots::Time& t)     
    throw (Tribots::TribotsException) { 
    LOUT<<"% white thick "<<Vec(-4000.,changeLine)<<" "<<Vec(4000.,changeLine)<<endl;
    return BDIBehavior::getCmd(t);
  }

  virtual bool checkCommitmentCondition(const Time& t) throw() {
//    const BallLocation& ballLocation = MWM.get_ball_location(t);
    //if ( (ballLocation.pos.toVec().y>0)&&(fieldplayer_role == "receiver") )
    if ( (fieldplayer_role == "receiver") )
      return true;
    
    return false; 
  }

  virtual bool checkInvocationCondition(const Time& t) throw() {
    return checkCommitmentCondition(t);

    return true;
  }  

  virtual void loseControl(const Time&) throw(TribotsException) 
  { counter = 0; }
};   


// -------------------------------------------------------------------------------------------------------------------------------


static const char *_tribots_fp_roles[3] = { "shooter", "receiver", "hold" };

ChallengePlayer::ChallengePlayer (const ConfigReader&) throw ()
  : BehaviorPlayer ("ChallengePlayer", _tribots_fp_roles, 2) {
  const FieldGeometry& fg (MWM.get_field_geometry());
  
  Vec home_pos (0, -0.5*fg.field_length+10);
  Vec right_end (0.5*fg.goal_width-300, -0.5*fg.field_length);
  Vec attack_area1 (-0.5*fg.goal_width, home_pos.y-200);
  Vec attack_area2 (0.5*fg.goal_width, home_pos.y+500);
  
  addOption (new BGameStopped);
  addOption (new BReceiverStack);
  addOption (new BShooterStack);
  addOption (new BGoBack);
  addOption (new BEmergencyStop);
  
}

ChallengePlayer::~ChallengePlayer () throw () {;}

bool
ChallengePlayer::set_role(const char* role) throw ()
{
  if (MultiRolePlayer::set_role(role)) {
    fieldplayer_role = std::string(role);      // fuer "interne" behaviors
    return true;
  }
  return false;
}
