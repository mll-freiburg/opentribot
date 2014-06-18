
#ifndef Tribots_BTestStateStop_h
#define Tribots_BTestStateStop_h

#include "BGameStopped.h"

namespace Tribots {

  /** Haelt den Roboter bei den Refereestate testStateN an. */
  class BTestStateStop : public BGameStopped {
  public:
    BTestStateStop () throw ();
    bool checkInvocationCondition(const Time&) throw();
    bool checkCommitmentCondition(const Time&) throw();
  };

}

#endif
