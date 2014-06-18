#ifndef Tribots_HumanCoopPlayer_h
#define Tribots_HumanCoopPlayer_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"


namespace Tribots {

  /** Feldspieler, verhaltensbasiert */
  class HumanCoopPlayer : public BehaviorPlayer {
  public:
    HumanCoopPlayer (const ConfigReader&) throw ();
    ~HumanCoopPlayer () throw ();

    bool set_role (const char*) throw ();
  };

}

#endif
