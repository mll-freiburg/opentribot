
#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"
#include "PlayerFactory.h"
#include "../WorldModel/WorldModel.h"
#include "../Behavior/Predicates/freeCorridor.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallToGoal.h"
#include "../Behavior/Behaviors/ApproachingBall/BComplexApproachBallFreePlay.h"
#include "../Behavior/Behaviors/BallHandling/BDribbleBallStraightToGoalEvadeSidewards.h"
#include "../Behavior/Skills/WithoutBall/SPatrol.h"
#include "../Behavior/BDIBehavior.h"
#include "../Behavior/Behaviors/BasicMovements/BShootEmergency.h"
#include "../Behavior/Behaviors/BasicMovements/BShoot.h"
#include "../Behavior/Behaviors/BallHandling/BEigenMove.h"

using namespace Tribots;
using namespace std;

namespace {
  const unsigned int nnn=3500; 
}

namespace Tribots {
  
  class SternBehavior : public Behavior {
  private:
    unsigned int step;
    unsigned int time_step_maximal;
    Time step_activated;
  public:
    SternBehavior () : Behavior ("SternBehavior"), step(0), time_step_maximal (nnn) {;}
    bool checkCommitmentCondition(const Time&) throw() {
      return (MWM.get_game_state().refstate==freePlay && MWM.get_game_state().in_game && step<4);
    }
    bool checkInvocationCondition(const Time& t) throw() {
      return checkCommitmentCondition(t);
    }
    void gainControl(const Time&) throw(TribotsException) {
      step_activated.update();
      time_step_maximal=nnn;
      MWM.reset ();
    }
    DriveVector getCmd(const Time& t) throw(TribotsException) {
      unsigned int tt = step_activated.elapsed_msec();
      if (tt>time_step_maximal) {
        step++;
        time_step_maximal = (step%2 ? tt : nnn);
        step_activated.update();
        LOUT << "Sternbehavior: Richtungswechsel wegen Zeitueberschreitung\n";        
      }
      DriveVector v (Vec(0,0), 0, false);
      if (step==0) {
        v.vtrans.y=1.0;
      } else if (step==1) {
        v.vtrans.y=-1.0;
      } else if (step==2) {
        v.vtrans.y=-1.0;
      } else if (step==3) {
        v.vtrans.y=1.0;
      }
      double dto = distance_to_obstacles (MWM.get_robot_location (t).pos, MWM.get_robot_location (t).heading+v.vtrans.angle(), 400, MWM.get_obstacle_location (t));
      if (dto<v.vtrans.length()*0.1+300) {
        step++;
        time_step_maximal = (step%2 ? tt : nnn);
        step_activated.update();
        v.vtrans.y=0;
        LOUT << "Sternbehavior: Richtungswechsel wegen Hindernis\n";        
      }
      return v;
    }
    void loseControl(const Time& t) throw(TribotsException) {
      LOUT << "Ende Sternfahrt: ";
      if (MWM.get_robot_location (t).pos.y>0) {
        LOUT << "drehe Orientierung\n";
        MWM.set_own_half (-MWM.get_own_half());
      } else {
        LOUT << "Orientierung okay\n";
      }
    }
    void cycleCallBack(const Time&) throw() {
      if (MWM.get_game_state().refstate==stopRobot) {
        step=0;
        MWM.set_own_half (+1);
      }
      LOUT << "CCB: " << step << '\n';
    }
  };
  
  
  class AngriffsBehavior :  public BDIBehavior {
    public:   
      AngriffsBehavior (const ConfigReader&) : BDIBehavior ("AngriffsBehavior") {
        addOption (new BShootEmergency());
        addOption (new BShoot());
        addOption (new BEigenMove());
        addOption (new BDribbleBallStraightToGoalEvadeSidewards());
        addOption (new BDribbleBallToGoal());
        addOption (new BComplexApproachBallFreePlay());
      };
      bool checkCommitmentCondition(const Time& t) throw() {
        return (MWM.get_game_state().refstate==freePlay && MWM.get_game_state().in_game && MWM.get_ball_location (t).pos_known==BallLocation::known);
      }
      bool checkInvokationCondition(const Time& t) throw() {
        return checkCommitmentCondition (t);
      }
  };
    

  class BallSucheBehavior : public Behavior {
  private:
    SPatrol* patrol;
  public:
    BallSucheBehavior () : Behavior ("BallSucheBehavior") {
      const FieldGeometry& fg = MWM.get_field_geometry ();
      vector<Vec> ppts (4);
      ppts [0] = Vec (-0.25*fg.field_width, 0.5*fg.field_length-3000);
      ppts [1] = Vec (-0.25*fg.field_width, 3000);
      ppts [2] = Vec (+0.25*fg.field_width, 3000);
      ppts [3] = Vec (+0.25*fg.field_width, 0.5*fg.field_length-3000);
      patrol = new SPatrol;
      patrol->setPatrolPositions (ppts);
    }
    ~BallSucheBehavior () throw () {
      delete patrol;
    }
    bool checkCommitmentCondition(const Time& t) throw() {
      return MWM.get_game_state().refstate==freePlay && MWM.get_game_state().in_game && MWM.get_ball_location(t).pos_known!=BallLocation::known;
    }
    bool checkInvocationCondition(const Time& t) throw() {
      return checkCommitmentCondition(t);
    }
    DriveVector getCmd(const Time& t) throw(TribotsException) {
      return patrol->getCmd (t);
    }
  };

  class StoppBehavior : public Behavior {
    public:
      StoppBehavior () : Behavior ("StopBehavior") {;}
      bool checkInvocationCondition (const Time&) throw () {
        return (!MWM.get_game_state().in_game || (MWM.get_game_state().refstate!=freePlay));
      }
      bool checkCommitmentCondition (const Time&) throw () {
        return (!MWM.get_game_state().in_game || (MWM.get_game_state().refstate!=freePlay));
      }
      DriveVector getCmd(const Time&) throw(TribotsException) {
        DriveVector null (Vec(0,0),0,false);
        return null;
      }
  };
  
  class ChallengePlayer07 : public BehaviorPlayer {
  public:
    ChallengePlayer07 (const ConfigReader& cfg) : BehaviorPlayer ("ChallengePlayer07") {
      addOption (new StoppBehavior);
      addOption (new SternBehavior);
      addOption (new BallSucheBehavior);
      addOption (new AngriffsBehavior (cfg));
    }
  };

}


  
// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
    public:
      Builder () {
        PlayerFactory::get_player_factory ()->sign_up (string("ChallengePlayer07"), this);
      }
      PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,std::bad_alloc) {
        return new ChallengePlayer07 (reader);
      }
  };
  Builder the_builder;
}
