
#ifndef _Tribots_BallFilter_h_
#define _Tribots_BallFilter_h_

#include "../../Structures/RobotLocation.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/VisibleObject.h"

namespace Tribots {

  /** Klasse BallFilter realisiert eine abstrakte Oberklasse fuer verschiedene Ballfilter */
  class BallFilter {
  public:
    /** Destruktor */
    virtual ~BallFilter () throw () {;}
    
    /** uebergebe gemessene Position an Filter;
        arg1: geshene Ballposition
        arg2: Roboterposition und -ausrichtung zum Zeitpunkt der Bildinformation 
        ret: wurde wirklich eine Aktualisierung vorgenommen? */
    virtual bool update (const VisibleObjectList&, const RobotLocation&) throw () =0;

    /** liefere mutmassliche Position und Geschwindigkeit zum Zeitpunkt arg1 */
    virtual BallLocation get (const Time) const throw () =0;
  };

}

#endif
