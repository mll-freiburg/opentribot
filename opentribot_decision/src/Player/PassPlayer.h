#ifndef Tribots_PassPlayer_h
#define Tribots_PassPlayer_h

#include "BehaviorPlayer.h"
#include "../WorldModel/WorldModel.h"
#include "../Fundamental/ConfigReader.h"

namespace Tribots {

  class PassPlayer:public BehaviorPlayer {
  public:
    PassPlayer (const ConfigReader&) throw ();
    //~PassPlayer();
  };

}

#endif

