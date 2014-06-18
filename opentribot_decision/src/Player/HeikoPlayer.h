#ifndef Tribots_FieldPlayer07_h
#define Tribots_FieldPlayer07_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"


namespace Tribots {

  /** Feldspieler, verhaltensbasiert */
  class HeikoPlayer : public BehaviorPlayer {
  public:
    HeikoPlayer (const ConfigReader&) throw ();
    ~HeikoPlayer () throw ();

    bool set_role (const char*) throw ();
  };

}

#endif
