#ifndef Tribots_PlayerTipkick_h
#define Tribots_PlayerTipkick_h

#include "BehaviorPlayer.h"
#include "../WorldModel/WorldModel.h"
#include "../Fundamental/ConfigReader.h"

namespace Tribots {

  /** Tipkick-Implementierung eines Spielertyps; */
  class PlayerTipkick:public BehaviorPlayer {
  public:
    PlayerTipkick (const ConfigReader&) throw ();
    //~PlayerTipkick();
  };

}

#endif

