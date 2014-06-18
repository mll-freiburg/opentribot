
#ifndef Tribots_RobotPositionPredictor_h_
#define Tribots_RobotPositionPredictor_h_

#include "../../Structures/RobotLocation.h"
#include "../../Fundamental/Time.h"

namespace Tribots {

  /** Klasse, um eine kurzfristige Vorhersage der Roboterposition durchzufuehren */
  class RobotPositionPredictor {
  private:
    RobotLocation current_rloc;       ///< Ausgangssituation
    Time timestamp_current_rloc;    ///< Zeitpunkt der Ausgangsposition
    Time timestamp_calculation;    ///< letzter Sensorintegrationsschritt
  public:
    /** Konstruktor */
    RobotPositionPredictor () throw ();
    /** uebergeben von neuer Information;
	arg1: neue Position, Geschwindigkeit, Kicker und Basisqualitaet der Schaetzung
	arg2: Zeitpunkt, den arg1 repraesentiert
	arg3: letzter Zeitpunkt, zu dem ein Sensorintegrationsschritt erfolgt ist */
    void set (const RobotLocation&, Time, Time) throw ();
    /** berechnen einer Position und Geschwindigkeit zu Zeitpunkt arg1 */
    RobotLocation get (Time) const throw ();
  };

}

#endif
