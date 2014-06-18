
#ifndef Tribots_ChallengePlayer_h
#define Tribots_ChallengePlayer_h

#include "../Fundamental/ConfigReader.h"
#include "BehaviorPlayer.h"


namespace Tribots {

  /** Player, um die Challenge 2006 auszuführen */
  class ChallengePlayer : public BehaviorPlayer {
  public:
    ChallengePlayer (const ConfigReader&) throw ();
    ~ChallengePlayer () throw ();
    
    bool set_role (const char*) throw ();  
  };

}

#endif
