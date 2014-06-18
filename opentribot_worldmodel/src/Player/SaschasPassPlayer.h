#ifndef _Tribots_SaschasPassPlayer_h_
#define _Tribots_SaschasPassPlayer_h_

#include "SingleRolePlayer.h"
#include "../Fundamental/ConfigReader.h"

namespace Tribots {

  /**
   * Spielertyp, der im Tutorial verwendet wird. Reagiert auf game-stopped.
   */
  class SaschasPassPlayer : public SingleRolePlayer {
  public:
    /** Konstruktor. Wird bei Instanzierung eines TutorialPlayers aufgerufen.
     */
    SaschasPassPlayer (const ConfigReader&) throw ();
    /** Destruktor. Wird bei der Zerstörung einer Instanz von TutorialPlayer
     *  aufgerufen */
    ~SaschasPassPlayer () throw () {}

    /** Berechnet einen Fahrvektor für den Zeitpunkt t */
    DriveVector process_drive_vector (Time t) throw ();
    
  protected:
    Time lastKick;
  };

}

#endif

