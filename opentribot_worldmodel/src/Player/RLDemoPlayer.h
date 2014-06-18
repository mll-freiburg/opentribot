#ifndef Tribots_RLDemoPlayer_h
#define Tribots_RLDemoPlyer_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"


namespace Tribots {

  /** Feldspieler, verhaltensbasiert */
  class RLDemoPlayer : public BehaviorPlayer {
  public:
    RLDemoPlayer (const ConfigReader&) throw ();
    ~RLDemoPlayer () throw ();

    bool set_role (const char*) throw ();
  };

}

#endif
