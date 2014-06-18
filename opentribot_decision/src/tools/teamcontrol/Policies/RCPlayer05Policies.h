
#ifndef TribotsTools_RCPlayerPolicies_h
#define TribotsTools_RCPlayerPolicies_h

#include "StaticPolicy.h"
#include "DynamicDefendOffendPolicy.h"
#include "../../../Fundamental/ConfigReader.h"

namespace RCPlayer05 {

  /** Strategie fuer RCPlayer mit Defend1, Defend2, Attack2 */
  class RCPlayerDDAPolicy : public TribotsTools::StaticPolicy {
  public:
    RCPlayerDDAPolicy (const Tribots::ConfigReader&);
    ~RCPlayerDDAPolicy () throw () {;}
  };

  /** Strategie fuer RCPlayer mit Defend3, Support, Attack2 */
  class RCPlayerDSAPolicy : public TribotsTools::StaticPolicy {
  public:
    RCPlayerDSAPolicy (const Tribots::ConfigReader&);
    ~RCPlayerDSAPolicy () throw () {;}
  };

  /** Strategie fuer RCPlayer, die dynamisch zwischen DDA und DSA umschaltet */
  class RCPlayerDDSAPolicy : public TribotsTools::DynamicDefendOffendPolicy {
  public:
    RCPlayerDDSAPolicy (const Tribots::ConfigReader&);
    ~RCPlayerDDSAPolicy () throw () {;}
  };

}

#endif
