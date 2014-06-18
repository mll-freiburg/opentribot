#ifndef _B_BOOST_BALL_TO_GOAL_H_
#define _B_BOOST_BALL_TO_GOAL_H_

#include "../../Behavior.h"
#include "../../Skills/BasicMovements/SBoostToPos.h"

namespace Tribots
{
  /** 
   * Bei freier Sicht auf Tor und Ausrichtung auf Tor wird mit
   * maximaler Geschwindigkeit der Ball Richtung Tor gebracht
   * benutzt: SBoostBallToPos
   */
  class BBoostBallToGoal : public Tribots::Behavior
  {
  public:
    BBoostBallToGoal();
    virtual ~BBoostBallToGoal() throw();
    
    /** Roboter hat den Ball, freie Bahn, Ausrichtung auf Tor **/
    virtual bool checkCommitmentCondition(const Time&) throw();
    virtual bool checkInvocationCondition(const Time&) throw();
    
    virtual DriveVector getCmd(const Time&) throw(TribotsException);
    virtual void gainControl(const Time&) throw(TribotsException);
    virtual void loseControl(const Time&) throw(TribotsException);   
    virtual void updateTactics (const TacticsBoard&) throw ();  
    virtual void cycleCallBack(const Time& t) throw();

  protected:
    SBoostToPos*	skill;
    double		exec_prob;
    bool		last_cycle_ball_possession;
    bool		licence;
  };
  
};

#endif
