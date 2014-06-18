
#ifndef _Tribots_BTestAllDirections_h_
#define _Tribots_BTestAllDirections_h_

#include "../../Behavior.h"
#include "../../../Fundamental/Angle.h"
#include "../../../Structures/GameState.h"

namespace Tribots {

  /** EIn Verhalten, das in alle drei symetrische Richtungen faehrt,
      um zu testen, ob ein Rad blockiert ist */
  class BTestAllDirections : public Behavior {
  private:
    RefereeState testState;
    RefereeState latestState;
    double msecDir [3];
    Angle angleDir [3];
    unsigned int latestDir;
    Time latestTimestamp;
    unsigned int msecMax;
    unsigned int waitCycle;
    int half;

    bool wayFree (Time, Angle, bool);

  public:
    BTestAllDirections (RefereeState);
    ~BTestAllDirections () throw ();
    bool checkInvocationCondition(const Time&) throw();
    bool checkCommitmentCondition(const Time&) throw();
    DriveVector getCmd(const Time&) throw();
    void loseControl (const Time&) throw(TribotsException);
    void gainControl (const Time&) throw(TribotsException);
    void cycleCallBack(const Time&) throw ();
  };

}

#endif
