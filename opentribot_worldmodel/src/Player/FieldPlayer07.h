#ifndef Tribots_FieldPlayer07_h
#define Tribots_FieldPlayer07_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"


namespace Tribots {

  /** Feldspieler, verhaltensbasiert */
  class FieldPlayer07 : public BehaviorPlayer {
  public:
    FieldPlayer07 (const ConfigReader&) throw ();
    ~FieldPlayer07 () throw ();

    bool set_role (const char*) throw ();
  };

}

#endif
