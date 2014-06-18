
#ifndef _Tribots_BTestStateStuckStop_h_
#define _Tribots_BTestStateStuckStop_h_

#include "../../../WorldModel/WorldModel.h"
#include "../BasicMovements/BEmergencyStop.h"

namespace Tribots {

  /** ein Verhalten, das wahrend der Testzustaende bei stuck den Roboter anhaelt.
      Achtung: Das Verhalten gibt die Kontrolle sofort wieder ab, sobald im Weltmodell
      stuck ausgeht */
  class BTestStateStuckStop : public BEmergencyStop {
  public:
    BTestStateStuckStop ();
    ~BTestStateStuckStop () throw ();
    bool checkInvocationCondition(const Time& t) throw();
    bool checkCommitmentCondition(const Time& t) throw();
  };

}

#endif
