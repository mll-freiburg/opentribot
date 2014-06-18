
#include "BGoaliePenalty.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../Skills/Goalie/SPhysGotoPosAvoidObstacles.h"
#include <cmath>

using namespace Tribots;
using namespace std;

namespace {

  /** vor dem Penalty in die Tomitte fahren */  
  class BGoaliePenaltyGotoPos : public Behavior {
    SPhysGotoPosAvoidObstacles goto_obs_skill;
    Vec home;
    int& balanceRightLeft;
  public:
    BGoaliePenaltyGotoPos (int& bRL) : Behavior ("BGoaliePenaltyGotoPos"), balanceRightLeft(bRL) {
      goto_obs_skill.set_dynamics(1.8);
      home.x=0;
      home.y=-0.5*MWM.get_field_geometry().field_length+200;
      goto_obs_skill.init (home, Angle::zero, true);
    }
    void gainControl (const Time&) throw () {
      home.x=100*balanceRightLeft;
      if (home.x>400) home.x=400;
      if (home.x<-400) home.x=-400;
      goto_obs_skill.init (home, Angle::zero, true);
      LOUT << "BGoaliePenaltyGotoPos: home.x=" << home.x << endl;
    }
    bool checkInvocationCondition (const Time&) throw () {
      return MWM.get_game_state().refstate==preOpponentPenalty;
    }
    bool checkCommitmentCondition (const Time& t) throw () {
      return MWM.get_game_state().refstate==preOpponentPenalty;
    }
    DriveVector getCmd(const Time& t) throw () {
      return goto_obs_skill.getCmd (t);
    }
  };

  /** so lange der Gegner noch nicht ausgefuehrt hat, stehen bleiben und die Ballposition beobachten */
  class BGoaliePenaltyWait : public Behavior {
    Vec& prePosition;
    double f;  // der Faktor fuer den gleitenden Durchschnitt
  public:
    BGoaliePenaltyWait (Vec& pp) throw () : Behavior ("BGoaliePenaltyWait"), prePosition(pp), f(1) {;}
    void gainControl (const Time&) throw (TribotsException) { f=1.0; }
    bool checkInvocationCondition (const Time&) throw () { return true; }
    bool checkCommitmentCondition (const Time& t) throw () {
      const BallLocation& ball (MWM.get_ball_location(t));
      return (f>0.09) || (ball.pos_known!=BallLocation::known) || (ball.pos.y+350>prePosition.y);
    }
    DriveVector getCmd(const Time& t) throw () {
      const BallLocation& ball (MWM.get_ball_location(t));
      if (ball.pos_known==BallLocation::known) {
        prePosition=f*ball.pos.toVec()+(1.0-f)*prePosition;
        f=f/(1.0+f);
        if (f<0.01)
          f=0.01;
      }
      DriveVector nullvec (Vec(0,0),0,false);
      return nullvec;
    }
  };

  /** je nach Ballposition zur Seite fahren */
  class BGoaliePenaltyDefend : public Behavior {
    Vec& prePosition;
    int& balanceRightLeft;
    Vec targetPosition;
    SPhysGotoPos goto_pos_skill;
    Time tb;
    unsigned int cycleCounterKick;
    unsigned int cycleCounterTotal;
  public:
    BGoaliePenaltyDefend (Vec& pp, int& bRL) : Behavior ("BGoaliePenaltyDefend"), prePosition(pp), balanceRightLeft(bRL) { goto_pos_skill.set_dynamics (2.0); }
    DriveVector getCmd(const Time& t) throw () {
      cycleCounterTotal++;
      const BallLocation& ball (MWM.get_ball_location(t));
      const FieldGeometry& fg (MWM.get_field_geometry());
      if (ball.pos_known==BallLocation::known && ball.pos.y+350<prePosition.y) {
        try{
          const FieldGeometry& fg (MWM.get_field_geometry());
          Line goalline (Vec(-1,-0.5*fg.field_length), Vec(1,-0.5*fg.field_length));
          Line ballline (prePosition, ball.pos.toVec());
          Vec ip = intersect (goalline, ballline);
          if (abs(ip.x)<0.5*fg.goal_width+2000) {
            double dy = (prePosition.y-ball.pos.y)/(prePosition.y-0.5*fg.field_length);
            double dx = (1.3-0.3*dy)*ip.x;  // beruecksichtigen, dass am Anfang der Winkel der Richtung eher unterschaetzt wird
            if (dx>0.5*fg.goal_width-300)
              dx=0.5*fg.goal_width-300;
            if (dx<-0.5*fg.goal_width+300)
              dx=-0.5*fg.goal_width+300;
            targetPosition.x=dx;
            targetPosition.y=-0.5*fg.field_length+200;
            tb=t;
            cycleCounterKick++;
            if (cycleCounterKick==6 && cycleCounterTotal<15) {
              // balanceRightLeft aktualisieren
              if (targetPosition.x>300)
                balanceRightLeft++;
              if (targetPosition.x<-300)
                balanceRightLeft--;
            }
          }
        }catch(invalid_argument&){;}
      }
      if (t.diff_msec(tb)>2000) {
        targetPosition.x=0;
        targetPosition.y=-0.5*fg.field_length+200;
      }
      LOUT << "% dark_red thick solid arrow " << prePosition << targetPosition << '\n';
      goto_pos_skill.init (targetPosition, Angle::zero, 1.0);
      DriveVector dest = goto_pos_skill.getCmd(t);
      return dest;
    }
    void gainControl (const Time&) throw (TribotsException) {
      targetPosition.x=0;
      targetPosition.y=-0.5*MWM.get_field_geometry().field_length;
      cycleCounterKick=0;
      cycleCounterTotal=0;
    }
    bool checkInvocationCondition (const Time&) throw () {
      return true;
    }
    bool checkCommitmentCondition (const Time&) throw () {
      return true;
    }
  };

}


BGoaliePenalty::BGoaliePenalty () throw () : SPBehavior ("BGoaliePenalty") {
  appendStage (new BGoaliePenaltyGotoPos (balanceRightLeft), false, false);
  appendStage (new BGoaliePenaltyWait (prePosition), false, false);
  appendStage (new BGoaliePenaltyDefend (prePosition, balanceRightLeft), false, false);
  balanceRightLeft=0;
}

bool BGoaliePenalty::checkInvocationCondition (const Time& t) throw () {
  return (MWM.get_game_state().refstate==preOpponentPenalty || 
          MWM.get_game_state().refstate==postOpponentPenalty || 
          MWM.get_game_state().refstate==opponentPenalty) &&
          SPBehavior::checkInvocationCondition (t);
}

bool BGoaliePenalty::checkCommitmentCondition (const Time& t) throw () {
  return (MWM.get_game_state().refstate==preOpponentPenalty || 
          MWM.get_game_state().refstate==postOpponentPenalty || 
          MWM.get_game_state().refstate==opponentPenalty) &&
          SPBehavior::checkCommitmentCondition (t);
}
