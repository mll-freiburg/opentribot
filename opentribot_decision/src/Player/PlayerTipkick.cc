#include "WhiteBoard.h"
#include "PlayerTipkick.h"
#include "PlayerFactory.h"
#include "../Behavior/Behaviors/SpecialGameStates/BGameStopped.h"
#include "../Behavior/Behaviors/BasicMovements/BEmergencyStop.h"
#include "../Behavior/Behaviors/WithoutBall/BOpposeBall.h"
#include "../Behavior/Behaviors/ApproachingBall/BInterceptBallStatic.h"
#include "../Behavior/Behaviors/ApproachingBall/BCatchBall.h"
#include "../Behavior/Behaviors/ApproachingBall/BTurnAroundPos.h"
#include "../Behavior/Behaviors/ApproachingBall/BApproachBallStatic.h"
#include "../Behavior/Behaviors/BasicMovements/BStayInsideArea.h"
#include "../Behavior/SPBehavior.h"


using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public PlayerBuilder {
    static Builder the_builder;
  public:
    Builder () {
      PlayerFactory::get_player_factory ()->sign_up (string("Tipkick"), this);
    }
    PlayerType* get_player (const std::string&, const ConfigReader& reader, PlayerType*) throw (TribotsException,bad_alloc) {
      return new PlayerTipkick (reader);
    }
  };
  Builder the_builder;
}


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



// should be the same like in FieldPlayer07.cc
// modified slightly to work without communicated pass-signal
class BallPassingReceiver : public SPBehavior {
private:

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
     appendStage(new BOpposeBall(false), false, true); // dont rely on communicated signal
     appendStage(new BInterceptBallStatic(), false, true);
     appendStage(new BBallTurning(), false, true);
  }
  
  ~BallPassingReceiver() throw () {
  }

  virtual bool checkCommitmentCondition(const Time& t) throw() {
    const BallLocation& ball = MWM.get_ball_location(t);    
    if (ball.pos_known == BallLocation::unknown) {
      LOUT << "BallPassingReceiver: Sorry, I can't see the ball. /n";
      return false;
    }
    return SPBehavior::checkCommitmentCondition(t);  
  }
};  // BallPassingReceiver finished 


PlayerTipkick::PlayerTipkick (const ConfigReader& reader) throw ()
  : BehaviorPlayer ("TipkickPlayer") 
{
  const FieldGeometry& fgeom = MWM.get_field_geometry();
  XYRectangle area(Vec( fgeom.field_width/2.  + 500., 
                        fgeom.field_length/2. + 500.),
                   Vec(-fgeom.field_width/2.  - 500.,
                       -fgeom.field_length/2. - 500)); 
  Vec target(0., fgeom.field_length/4.);


  WBOARD->readConfigs(reader);
  WBOARD->checkMessageBoard(); // important for getting ownsball

  addOption(new BGameStopped()); // react to the stop-signal
  addOption(new BStayInsideArea(area, target)); // Training

  // central behavior
  addOption (new BallPassingReceiver());

  addOption(new BEmergencyStop());
}

