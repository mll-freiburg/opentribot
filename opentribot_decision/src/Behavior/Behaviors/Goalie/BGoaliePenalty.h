
#ifndef Tribots_BGoaliePenalty_h
#define Tribots_BGoaliePenalty_h

#include "../../SPBehavior.h"

namespace Tribots {

  /** Positioniert den Goalie in der Mitte des Tores und faehrt auf der Linie in
      Abhaengigkeit von der Ballposition */
  class BGoaliePenalty : public SPBehavior {
    Vec prePosition;  ///< Die Ballposition vor Ausfuehrung des Strafstosses
    int balanceRightLeft;  ///< Die Anzahl Einschuesse rechts minus Anzahl Einschuesse links
  public:
    BGoaliePenalty () throw ();
    bool checkInvocationCondition (const Time&) throw ();
    bool checkCommitmentCondition (const Time&) throw ();
  };

}

#endif 
