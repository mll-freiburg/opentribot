#ifndef _Tribots_TutorialPlayer_h_
#define _Tribots_TutorialPlayer_h_

#include "SingleRolePlayer.h"
#include "../Fundamental/ConfigReader.h"

namespace Tribots {

  /**
   * Spielertyp, der im Tutorial verwendet wird. Reagiert auf game-stopped.
   */
  class TutorialPlayer : public SingleRolePlayer {
  public:
    /** Konstruktor. Wird bei Instanzierung eines TutorialPlayers aufgerufen.
     */
    TutorialPlayer (const ConfigReader&) throw ();
    /** Destruktor. Wird bei der Zerstörung einer Instanz von TutorialPlayer
     *  aufgerufen */
    ~TutorialPlayer () throw () {}

    /** Berechnet einen Fahrvektor für den Zeitpunkt t */
    DriveVector process_drive_vector (Time t) throw ();
  protected:
    bool darfkicken;

  };

}

#endif

