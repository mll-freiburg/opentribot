#include "WhiteBoard.h"
#include "../WorldModel/WorldModel.h"
#include "ChallengePlayer08.h"
#include "PlayerFactory.h"

#include "../Behavior/Behaviors/ApproachingBall/BExplore.h"

#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BallHandling/BPass.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/Behaviors/BasicMovements/BGotoPosEvadeObstacles.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallDirectly.h"

#include "../Behavior/Behaviors/ApproachingBall/BComplexApproachBallFreePlay.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallFromBehindPointingToGoal.h"
#include "../Behavior/Behaviors/ApproachingBall/BVolleyApproach.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToGoal.h"
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
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BasicMovements/BGotoPosEvadeObstacles.h"
#include "../Behavior/Behaviors/WithoutBall/BSupportLongPass.h"
#include "../Behavior/Behaviors/WithoutBall/BSupportNearBall.h"
#include "../Behavior/Behaviors/WithoutBall/BCounterAttack.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/SpecialGameStates/BTestStateStop.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOpponentStandardSituationNew.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPostOpponentStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BPreOwnIndirectStandardSituation.h"
#include "../Behavior/Behaviors/SpecialGameStates/BOwnPenalty.h"
#include "../Behavior/Behaviors/ZonePressure/BZonePressure.h"
#include "../Behavior/Behaviors/WithoutBall/BLeaveGoal.h"
#include "../Behavior/Behaviors/WithoutBall/BOpposeBall.h"
#include "../Behavior/Behaviors/ApproachingBall/BInterceptBallStatic.h"
#include "../Behavior/Behaviors/ApproachingBall/BCatchBall.h"
#include "../Behavior/Behaviors/ApproachingBall/BTurnAroundPos.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallStatic.h"

#include <cmath>
#include <vector>


#include "../ImageProcessing/ObjectAnalysis/Regions.h"
//#include "../ImageProcessing/ObjectAnalysis/ChainCoding.h"

namespace Tribots {
  //stages
  int captureFirstPartOfDiff = 0;
  int captureSecondPartOfDiff = 0;
  int captureRegionColors = 0;
  bool ballAnalysis = false;
  bool searchForBall = false;

  //shared storage
  Tribots::RegionList * oldlist;
  Tribots::RegionList * possibleNewRegions;
  std::vector<int> newRegionCount;
  Tribots::Region *favRegion = NULL;
}

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("ChallengePlayer08"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new ChallengePlayer08 (reader);
    }
  };
  Builder the_builder;
}

class BExploreConditioned : public BExplore {
public:
  BExploreConditioned() : BExplore("BExplore") {}
  
  bool checkCommitmentCondition(const Time& t) throw() {
    return checkInvocationCondition(t);
  }

  bool checkInvocationCondition(const Time& t) throw() {
    return BExplore::checkInvocationCondition() && searchForBall;
  }
};

class BVision : public Behavior {
private:
  int oldstate;
public:
  BVision() : Behavior("BVision", true), oldstate(-1) {
}
  
  bool checkCommitmentCondition(const Time& t) throw() {
    return checkInvocationCondition(t);
  }

  bool checkInvocationCondition(const Time& t) throw() {
    return false;
  }

  void cycleCallBack(const Time& t) throw() {
    int gamestate = WorldModel::get_main_world_model().get_game_state().refstate;

    LOUT << "gamestate: " << gamestate << endl;

    if (gamestate == stopRobot) {
	if (oldstate != stopRobot) {
	  oldstate = gamestate;
	  LOUT << "ChallengePlayer08: Reset?" << endl;

	  captureFirstPartOfDiff = 0;
          captureSecondPartOfDiff = 0;
        }
    }

    if (gamestate == preDroppedBall) {
	if (oldstate != preDroppedBall) {
	  oldstate = gamestate;
	  LOUT << "ChallengePlayer08: Taking fist part of region diff" << endl;
	  //create part one of diff region map
	  captureFirstPartOfDiff = 1;
        }
    }

    if (gamestate == freePlay) {
	if (oldstate != freePlay) {
	  oldstate = gamestate;
          LOUT << "ChallengePlayer08: Second part of region diff" << endl;
	  //now second image can be taken
	  captureSecondPartOfDiff = 1;
       }
    }

    if (gamestate == preOwnKickOff) {
	if (oldstate != preOwnKickOff) {
	  oldstate = gamestate;
          LOUT << "ChallengePlayer08: Kickoff, starting to search the ball" << endl;
          searchForBall=true;
       }
    }
  }

  DriveVector getCmd(const Time& time) throw(TribotsException) {
    //stand ground
    DriveVector dv (Vec(0,0), 0, false);
    return dv;
  }
};

class BallOwnerStack : public BDIBehavior {
private:
  int counter;          ///< zur Ueberbrueckung weniger Zyklen ohne Ballbesitz

  /**
   * Ueberpruefung allgemeiner Bedingungen.
   */
  virtual bool checkConditions(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);    
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
  BallOwnerStack(int hackKickLength = 120) 
    : BDIBehavior("ChallengePlayer08BallOwnerStack")
  {
    addOption (new BShootEmergency(hackKickLength));
    addOption (new BShoot(hackKickLength));
    addOption (new BEigenMove());
    addOption (new BShakeOffDefender());
    addOption (new BBoostBallToGoal());
    addOption (new BDribbleBallStraightToGoalEvadeSidewards());
    addOption (new BDribbleBallToGoal());
    addOption (new BComplexApproachBallFreePlay());
  }
  
  ~BallOwnerStack() throw () {
  }

  /**
   * Generiert einen DriveVector. Verwendet dazu die unveraenderte Methode
   * des BDI-Verhaltens (siehe BDIBehavior), ueberprueft aber vorher, ob
   * beim Teamcontrol die Rolle "ball" aktiv beantragt werden soll.
   */
  virtual DriveVector getCmd(const Tribots::Time& t) throw (Tribots::TribotsException) { 
    return BDIBehavior::getCmd(t);
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
      LOUT << "BallStack: Den " << 31-counter << ". Zyklus keinen Ballbesitz mehr." << endl;
      --counter;
      return true;
    }
    const BallLocation& ball = MWM.get_ball_location(t);
    return true;   
  }               

  virtual bool checkInvocationCondition(const Time& t) throw() {
    if (!checkConditions(t)) {
      return false;
    }
    return true;
  }  

  virtual void loseControl(const Time&) throw(TribotsException) 
  { counter = 0; }
};



ChallengePlayer08::ChallengePlayer08 (const ConfigReader& reader) throw ()
  : BehaviorPlayer ("ChallengePlayer08") {

  WBOARD->readConfigs (reader);
  WBOARD->checkMessageBoard(); // important for getting ownsball

  const FieldGeometry& fgeom = MWM.get_field_geometry();
  XYRectangle area(Vec( fgeom.field_width/2.  + 500., 
                        fgeom.field_length/2. + 500.),
                   Vec(-fgeom.field_width/2.  - 500.,
                       -fgeom.field_length/2. - 500)); 
  Vec target(0.0, fgeom.field_length/4.);
  Vec startPos(fgeom.field_width/3., -fgeom.field_length/3.);
	Vec startHeading(0.0, 1.0);

  addOption(new BGameStopped());
  addOption(new BVision());           //only to tell vision the game state
  addOption(new BExploreConditioned());
  addOption(new BallOwnerStack());

//  addOption(new BStayInsideArea(area, target)); // Training
//  addOption(new BPass(1.0)); //pass probability
//  addOption(new BApproachBallDirectlyConditioned());
  addOption(new BEmergencyStop());
}
