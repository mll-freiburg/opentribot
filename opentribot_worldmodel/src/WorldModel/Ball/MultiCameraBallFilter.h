
#ifndef _Tribots_MultiCameraBallFilter_h_
#define _Tribots_MultiCameraBallFilter_h_

#include "BallFilter.h"
#include "../Orga/OdometryContainer.h"
#include "../../Fundamental/ConfigReader.h"
#include <vector>

namespace Tribots {

  /** Klasse MultiCameraBallFilter kombiniert Ballfilter fuer mehrere Bildquellen.
      2 Modi: die gesehenen Baelle aller Bildquellen werden in einem gemeinsamen Filter verarbeitet (common),
      oder fuer jede Bildquelle wird individueller Ballfilter verwendet und nach Prioritaeten angewendet */
  class MultiCameraBallFilter {
  public:
    /** erzeuge BallFilter; lese Parameter aus arg1 */
    MultiCameraBallFilter (const ConfigReader&, const OdometryContainer&) throw (std::bad_alloc);
    /** Destruktor */
    ~MultiCameraBallFilter () throw ();
    /** uebergebe neue Bildinformation an Filter, getrennt nach Bildquellen;
        arg1=gesehener Ballposition, 
        arg2=Roboterposition und -ausrichtung zum Zeitpunkt der Bildinformation
        ret=Wurde tatsaechliche eine Aktualisierung durchgefuehrt? */
    bool update (const std::vector<VisibleObjectList>&, const std::vector<RobotLocation>&) throw ();
    /** Kommunizierten Ball uebergeben (in Weltkoordinaten) */
    void comm_ball (Vec) throw ();
    /** liefere mutmasliche Position und Geschwindigkeit zum Zeitpunkt arg1 */
    BallLocation get (const Time) const throw ();

  private:
    std::vector<BallFilter*> individual_ball_filter;   // Ballfilter fuer jede Bildquelle individuell
    BallFilter* common_ball_filter;  // Ballfilter fuer alle Kameras gemeinsam
    std::vector<unsigned int> imagesources;  // Prioritatsliste der Bildquellen
    bool common_mode;  // Modus: true->nehme common_ball_filter; false->nutze individual_ball_filter

    const ConfigReader& cfg;
    const OdometryContainer& odobox;

    Vec comm_ball_pos;           // letzter Kommunizierter Ball
    Time comm_ball_pos_time;     // Zeitpunkt der letzten Kommunikation
  };

}

#endif
