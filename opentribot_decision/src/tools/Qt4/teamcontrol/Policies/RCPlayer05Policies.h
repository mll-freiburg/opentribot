
#ifndef TribotsTools_RCPlayerPolicies_h
#define TribotsTools_RCPlayerPolicies_h

#include "StaticPolicy.h"
#include "DynamicDefendOffendPolicy.h"

namespace RCPlayer05 {

  /** Strategie fuer RCPlayer mit Defend1, Defend2, Attack2 */
  class RCPlayerDDAPolicy : public TribotsTools::StaticPolicy {
  public:
    RCPlayerDDAPolicy ();
    ~RCPlayerDDAPolicy () throw () {;}
  };

  /** Strategie fuer RCPlayer mit Defend3, Support, Attack2 */
  class RCPlayerDSAPolicy : public TribotsTools::StaticPolicy {
  public:
    RCPlayerDSAPolicy ();
    ~RCPlayerDSAPolicy () throw () {;}
  };

  /** Strategie fuer RCPlayer, die dynamisch zwischen DDA und DSA umschaltet */
  class RCPlayerDDSAPolicy : public TribotsTools::DynamicDefendOffendPolicy {
  public:
    RCPlayerDDSAPolicy ();
    ~RCPlayerDDSAPolicy () throw () {;}
  };

}

#endif
