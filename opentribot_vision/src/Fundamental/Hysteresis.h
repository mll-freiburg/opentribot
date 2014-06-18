
#ifndef _Tribots_Hysteresis_h_
#define _Tribots_Hysteresis_h_

namespace Tribots {

  /** Hysteresis implementiert eine klassische Hystherese mit unterem und oberem Schwellwert.
      Der indifferente Bereich ist gegeben durch das Intervall (lowerThreshold, upperThreshold).
      Die beiden Zustaende der Hystherese werden mit false (niedrig) und true (hoch) beschrieben */
  class Hysteresis {
    double lowerThreshold;
    double upperThreshold;
    bool isActive;
  public:
    /** Initialisieren mit lt=unterer Schwellwert, ut=oberer Schwellwert, s=Initialzustand */
    Hysteresis (double lt = 0, double ut =1, bool s =false) throw () : lowerThreshold(lt), upperThreshold(ut), isActive(s) {;}
    /** Zustand abfragen */
    bool get () const throw () { return isActive; }
    /** Zustand aktualisieren und aktualisierten Zustand zureuckgeben */
    bool update (double v) throw () { isActive = (v>=upperThreshold || (isActive && v>lowerThreshold)); return isActive; }
    /** Thresholds setzen */
    void setThreshold (double lt, double ut) throw () { lowerThreshold=lt; upperThreshold=ut; }
    /** Internen Zustand der Hysterese setzen */
    void set (bool s) throw () { isActive=s; }
  };

}

#endif
