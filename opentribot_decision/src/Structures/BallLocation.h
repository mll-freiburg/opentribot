
#ifndef _Tribots_BallLocation_h_
#define _Tribots_BallLocation_h_

#include "../Fundamental/Vec.h"
#include "../Fundamental/Vec3D.h"
#include "../Fundamental/Time.h"

namespace Tribots {

  /** Struktur, um die derzeitige Ballposition auf dem Spielfeld zu repraesentieren  
      alle Koordinaten (in WELTKOORDINATEN) sind relativ zur aktuellen Spielrichtung:
      (0,0) ist der Mittelpunkt des Feldes
      die y-Achse zeigt in Richtung des gegnerischen Tores
      die x-Achse zeigt rechtwinklig dazu (rechtshaendisches Koordinatensystem)
      alle Laengenangaben in mm, alle Geschwindigkeiten in m/s */
  struct BallLocation {
    enum BallAttribute {
      unknown =0,       ///< Ballposition vollkommen unbekannt
      known =1,         ///< Ballposition hinreichend genau bekannt
      raised =2,        ///< Ball mutmaslich hochgeschossen oder hochgenommen
      communicated =4   ///< Ballposition von Mitspielern kommuniziert
    };

    Vec3D pos;            ///< Ballposition
    Vec3D velocity;       ///< Ballgeschwindigkeit in m/s

    Time lastly_seen;     ///< Zeitpunkt, zu dem der Ball zuletzt gesehen wurde
    BallAttribute pos_known;  ///< ein Attribut, das angibt, ob die Ballposition bekannt ist oder nicht, Werte aus nachfolgenden statischen Konstanten
    bool velocity_known;      ///< gibt an, ob die Geschwindigkeit bestimmt werden konnte
  };

}

#endif
