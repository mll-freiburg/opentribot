#ifndef _Tribots_ZickZackTestPlayer_h_
#define _Tribots_ZickZackTestPlayer_h_

#include "SingleRolePlayer.h"
#include "../Fundamental/ConfigReader.h"

namespace Tribots {

  /**
   * Spielertyp, der im Tutorial verwendet wird. Reagiert auf game-stopped.
   */
  class ZickZackTestPlayer : public SingleRolePlayer {
  public:
    /** Konstruktor. Wird bei Instanzierung eines ZickZackTestPlayers aufgerufen.
     */
    ZickZackTestPlayer (const ConfigReader&) throw ();
    /** Destruktor. Wird bei der Zerstˆrung einer Instanz von ZickZackTestPlayer
     *  aufgerufen */
    ~ZickZackTestPlayer () throw () {}

    /** Berechnet einen Fahrvektor f¸r den Zeitpunkt t */
    DriveVector process_drive_vector (Time t) throw ();
  protected:
    int state;
    int counter;
    Time timer;
  };

}

#endif

