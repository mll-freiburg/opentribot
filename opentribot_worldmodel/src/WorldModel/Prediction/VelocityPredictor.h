
#ifndef _Tribots_VelocityPredictor_h_
#define _Tribots_VelocityPredictor_h_

#include "../../Fundamental/ConfigReader.h"
#include "../../Fundamental/Time.h"
#include "../../Structures/RobotLocation.h"
#include "../../Structures/DriveVector.h"

namespace Tribots {

  /** Schnittstellenklasse fuer Geschwindigkeits-Vorhersagen */
  class VelocityPredictor {
  public:
    virtual ~VelocityPredictor () throw () {;}
    /** eine Positionsschaetzung einbinden,
        Arg1: Zeitpunkt, auf den sich die Schaetzung bezieht,
        Arg2: Positionschaetzung */
    virtual void notify_position (const RobotLocation&, Time) throw () =0;
    /** einen abgesetzten Fahrtvektor bekannt machen,
        Arg1: Zeitpunkt, zu dem der Fahrtvektor mutmasslich erste Auswirkungen hat,
        Arg2: Fahrtvektor */
    virtual void notify_drive_vector (DriveVector, Time) throw () =0;
    /** neue Geschwindigkeitsschaetzungen berechnen */
    virtual void update () throw () =0;
  };

}

#endif
