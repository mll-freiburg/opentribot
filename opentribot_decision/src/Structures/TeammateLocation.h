
#ifndef _Tribots_TeammateLocation_h_
#define _Tribots_TeammateLocation_h_

#include "RobotLocation.h"
#include "ObstacleLocation.h"
#include "../Fundamental/Time.h"
#include <iostream>

namespace Tribots {

  /** OccupancyGrid in Polarkoordinaten in der Umgebung eines Mitspielers.
      Anordnung der Zellen: es gibt zwei Ringe mit Zellen um den Roboter, jeder
      Ring ist in 8 Zellen a 45 Grad eingeteilt. Zellen 0..7 liegen im inneren
      Ring, Zellen 8..15 im aeusseren. Zelle 0 und 8 beginnen bei 0 Grad und 
      enden bei 45 Grad im Weltkoordinaten, die Numerierung der weiteren
      Zellen erfolgt im Gegenuhrzeigersinn. Der Zugriff auf die Zellen sollte
      nach Moeglichkeit ueber die occupied-Methoden erfolgen */
  struct TeammateOccupancyGrid {
    static const double inner_radius;  ///< der Radius des inneren Rings von Zellen
    static const double outer_radius;  ///< der Radius des aeusseren Rings von Zellen
    static const unsigned int num_cells; ///< Anzahl Zellen

    /** prueft, ob eine bestimmte Richtung (arg1) in Weltkoordinaten frei ist;
        Rueckgabewerte: 0=Richtung frei, 1=innere Zelle belegt, 
        2=aeussere Zelle belegt, 3=innere und aussere Zelle belegt */
    unsigned short int occupied (Angle) const throw ();
    /** prueft, ob ein Zwickel zwischen Richtung (arg1) und (arg2)
         in mathematisch positiver Drehrichtung in Weltkoordinaten frei ist.
         Rueckgabewerte: 0=Richtung frei, 1=belegt */
    unsigned short int occupied (Angle, Angle) const throw ();

    bool cells [16];  ///< Zelle i belegt oder teilweise belegt durch Hindernisse?

    TeammateOccupancyGrid () throw ();
    TeammateOccupancyGrid (const TeammateOccupancyGrid&) throw ();
    /** ein Occ-Grid berechnen. Roboterposition ist arg1, Hindernisliste n arg2 */
    TeammateOccupancyGrid (Vec, const ObstacleLocation&) throw ();
    const TeammateOccupancyGrid& operator= (const TeammateOccupancyGrid&) throw ();

    /** Visualisierung des Occ-Grid durch Schreiben einer "LOUT-Beschreibung" in arg1,
        arg2 ist die Roboterposition; fuer debug-Zwecke */
    void visualize (std::ostream&, Vec) throw (std::bad_alloc);
  };

  /** Struktur zur Repraesentation der Mitspielerpositionen */
  struct TeammateLocation : public RobotLocation {
    unsigned int number;   ///< Roboternummer
    Time timestamp;   ///< ungefaehrer Referenzzeitpunkt
    TeammateOccupancyGrid occupancy_grid;  ///< das zugehoerige OccupancyGrid
  };

}

#endif
