
#ifndef _Tribots_freeCorridor_h_
#define _Tribots_freeCorridor_h_

#include "../../Structures/ObstacleLocation.h"

namespace Tribots {

  /** liefert den minimalen Abstand der Hindernisse (arg3) zu der
      Verbindungslinie von (arg1) zu (arg2) in Millimeter */
  double obstacle_distance_to_line (Vec, Vec, const ObstacleLocation&, 
                                    bool ignoreTeammates=false) throw ();

  /** liefert den minimalen Abstand der Hindernisse (arg3) zu der
      Verbindungslinie von (arg1) zu (arg2) in Millimeter, beruecksichtige 
      nur Hindernisse innerhalb des Feldes */
  double obstacle_distance_to_line_inside_field (Vec, Vec, const ObstacleLocation&,
                                                 bool ignoreTeammates=false) throw ();

  /** berechnet, wie weit man von (arg1) in Richtung (arg2) gehen
      kann, ohne einem Hindernis (arg4) naeher als (arg3) Millimeter zu kommen;
      (=Laenge des freien Korridors von (arg1) in Richtung (arg2) mit Breite (arg3)) */
  double distance_to_obstacles (Vec, Angle, double, const ObstacleLocation&,
                                bool ignoreTeammates=false) throw ();

}

#endif
