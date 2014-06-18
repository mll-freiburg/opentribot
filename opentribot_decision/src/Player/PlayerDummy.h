
#ifndef Tribots_PlayerDummy_h
#define Tribots_PlayerDummy_h

#include "SingleRolePlayer.h"
#include "../WorldModel/WorldModel.h"
#include "../Fundamental/ConfigReader.h"

namespace Tribots {

  /** Dummy-Implementierung eines Spielertyps; tut nichts anderes als
      den Null-Fahrtvektor zu erzeugen. Bei Angabe von PlayerDummy::vtrans
      und PlayerDummy::vrot in der Konfigurationsdatei kann alternativ eine 
      Sollausgabe vorgegeben werden */
  class PlayerDummy:public SingleRolePlayer {
  private:
    Vec vtrans;
    double vrot;
    double vaux[3];
    DriveVectorMode dv_mode;
  public:
    PlayerDummy (const ConfigReader&) throw ();
    ~PlayerDummy () throw () {;}
    DriveVector process_drive_vector (Time tt) throw ();
  };

}

#endif

