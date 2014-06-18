
#ifndef _Tribots_ComPlayer_h_
#define _Tribots_ComPlayer_h_

#include "../PlayerType.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../Fundamental/Time.h"
#include "udpPlayerServer.h"

namespace Tribots {

  class ComPlayer : public PlayerType {
  public:
    /** Konstruktor */
    ComPlayer (const ConfigReader&) throw ();   // evtl. werden doch Exceptions geworfen
    /** Destruktor */
    ~ComPlayer () throw ();

    /** berechne Fahr- und Kickbefehl */
    DriveVector process_drive_vector (Time) throw ();
  private:
    udpPlayerServer serv;
  };

}

#endif

