#ifndef _Tribots_KickerDriveTest_h_
#define _Tribots_KickerDriveTest_h_

#include "SingleRolePlayer.h"
#include "../Fundamental/ConfigReader.h"

namespace Tribots {
	
  /**
   * Spielertyp, der im Tutorial verwendet wird. Reagiert auf game-stopped.
   */
  class KickerDriveTest : public SingleRolePlayer {
  public:
    /** Konstruktor. Wird bei Instanzierung eines TutorialPlayers aufgerufen.
     */
    KickerDriveTest (const ConfigReader&) throw ();
    /** Destruktor. Wird bei der Zerstˆrung einer Instanz von TutorialPlayer
     *  aufgerufen */
    ~KickerDriveTest () throw () {}
		
    /** Berechnet einen Fahrvektor f¸r den Zeitpunkt t */
    DriveVector process_drive_vector (Time t) throw ();
  protected:
    Time lastKick;
    bool kicked;
  };
	
}

#endif

