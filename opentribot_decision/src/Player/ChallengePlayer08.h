#ifndef Tribots_ChallengePlayer08_h
#define Tribots_ChallengePlayer08_h

#include "BehaviorPlayer.h"
#include "../WorldModel/WorldModel.h"
#include "../Fundamental/ConfigReader.h"

namespace Tribots {

  class ChallengePlayer08:public BehaviorPlayer {
  public:
    ChallengePlayer08 (const ConfigReader&) throw ();
  };

}

#endif

