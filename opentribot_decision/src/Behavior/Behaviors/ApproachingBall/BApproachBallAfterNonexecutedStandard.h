
#ifndef _Tribots_BApproachBallAfterNonexecutedStandard_h_
#define _Tribots_BApproachBallAfterNonexecutedStandard_h_

#include "../../Behavior.h"
#include "../../Skills/Goalie/SPhysGotoPos.h"

namespace Tribots {

  /**  faehrt Ball an nach nicht-ausgefuehrter Standardsituation */
  class BApproachBallAfterNonexecutedStandard : public Behavior {
  public:
    BApproachBallAfterNonexecutedStandard () throw ();
    ~BApproachBallAfterNonexecutedStandard() throw() {;}
    DriveVector getCmd(const Time&) throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
    void cycleCallBack(const Time&) throw ();
    void updateTactics (const TacticsBoard&) throw ();
    void loseControl(const Time&) throw(TribotsException);
    
  protected:
    SPhysGotoPos goto_pos_skill;
    Time post_standard_start;
    Vec ballpos_post;
    bool wait_for_free;
    bool active;
    bool active_over;
    double waittime;
  };

}

#endif 
