#ifndef Tribots_FieldPlayer_h
#define Tribots_FieldPlayer_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"


namespace Tribots {

  /** Feldspieler, verhaltensbasiert */
  class FieldPlayer : public BehaviorPlayer {
  public:
    FieldPlayer (const ConfigReader&) throw ();
    ~FieldPlayer () throw ();

    bool set_role (const char*) throw ();
  };

}

#endif
