#ifndef Tribots_SimpleDefender_h
#define Tribots_SimpleDefender_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"


namespace Tribots {

  /** Feldspieler, verhaltensbasiert */
  class SimpleDefender : public BehaviorPlayer {
  public:
    SimpleDefender (const ConfigReader&) throw ();
    ~SimpleDefender () throw ();

    bool set_role (const char*) throw ();
  };

}

#endif
