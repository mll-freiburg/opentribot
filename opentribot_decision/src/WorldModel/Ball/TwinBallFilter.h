
#ifndef Tribots_TwinBallFilter_h
#define Tribots_TwinBallFilter_h

#include "DynamicSlidingWindowBallFilter.h"
#include "OdometrySlidingWindowBallFilter.h"


namespace Tribots {

  /** Klasse TwinBallFilter realisiert ein Ensemble aus dem DynamicSlidingWindowBallFilter
      und OdometrySlidingWindowBallFilter, um die jeweiligen Fehler zu verringern;
      beherrscht z.Z. keine kommunizierte Baelle */
  class TwinBallFilter : public BallFilter {
  private:
    DynamicSlidingWindowBallFilter sl_filter;
    OdometrySlidingWindowBallFilter odo_filter;
  public:
    /** Konstruktor */
    TwinBallFilter (const ConfigReader&, const OdometryContainer&) throw (std::bad_alloc);

    /** uebergebe gemessene Position an Filter;
        arg1: geshene Ballposition
        arg2: Roboterposition und -ausrichtung zum Zeitpunkt der Bildinformation */
    void update (const VisibleObjectList&, const RobotLocation&) throw ();

    /** liefere mutmassliche Position und Geschwindigkeit zum Zeitpunkt arg1 */
    BallLocation get (const Time) const throw ();
  };

}

#endif
