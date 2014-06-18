
#ifndef Tribots_RadialObstacleView_h
#define Tribots_RadialObstacleView_h

#include <vector>
#include <stdexcept>
#include "../../Fundamental/Time.h"
#include "../../Fundamental/Vec.h"

namespace Tribots {

  /** fuer ein Hindernis bezogen auf die Roboterposition (nicht Ausrichtung!) die Winkel, in denen die Hindernisecken gesehen werden sowie Breite und Entfernung des Hindernisses */
  struct RadialObstacleView {
    Angle leftangle;  ///< Winkel in der der Punkt links aussen am Hindernis gesehen wird
    Angle mainangle;  ///< Winkel in der die Hinderniss-Mitte gesehen wird
    Angle rightangle;  ///< Winkel in der der Punkt rechts aussen am hindernis gesehen wird
    double distance;  ///< Abstand des Hindernis-Mittelunktes vom Roboter
    double width;  ///< Breite des Hindernisses
  };

  /** Eine Liste der Hindernisse berechnen:
     arg1: Rueckgabe
     arg2: Zeitpunkt der Auswertung
     arg3: den Ball als zusaetzliches Hindernis modellieren?
     arg4: die Spielfeldbegrenzungen als zusaetzliche Hindernisse eintragen?
     arg5: den Spielfeldrand (weisse Linie) als zusaetzliche Hindernisse eintragen?
     arg6: die Tore als Hindernisse eintragen? */
  void make_radial_obstacle_view (std::vector<RadialObstacleView>&, Time, bool =true, bool =true, bool =false, bool =true) throw (std::bad_alloc);

  /** Hinzufuegen eines Hindernisses:
     arg1: Liste, zu der das Hindernis hinzugefuegt werden soll
     arg2: Zeitpunkt der Auswertung
     arg3: Position des Hindernisses (Mitte)
     arg4: Breite des Hindernisses in mm */
  void add_obstacle (std::vector<RadialObstacleView>&, Time, Vec, double) throw (std::bad_alloc);
  
  /** RadialObstacleView in LOUT einzeichnen
    arg1: Hindernisse
    arg2: Fussposition (i.d.R. Roboterposition) */
  void visualize_radial_obstacle_view (const std::vector<RadialObstacleView>&, Vec) throw ();

  /** Auf einem Strahl die Liste der Hindernisse durchsuchen:
      arg1: Rueckgabe Referenz auf das Hindernis am naechsten links des Strahls (euklidscher Abstand)
      arg2: Rueckgabe  Referenz auf das Hindernis am naechsten rechts des Strahls (euklidscher Abstand)
      arg3: Rueckgabe Abstand zu den Hindernissen nach links in mm (>=100000, falls kein Hindernis gefunden)
      arg4: Rueckgabe Abstand zu den Hindernissen nach rechts in mm (>=100000, falls kein Hindernis gefunden)
      arg5: RadialObstacleView-Liste in der gesucht werden soll
      arg6: Hindernisse bis zu welchem Maximalabstand in mm beruecksichtigen?
      arg7: Strahlrichtung
      Return: >=100000, wenn kein Hindernis auf dem Strahl liegt, sonst der Abstand zu diesem Hindernis in mm */
  double scan_radial_obstacle_view (unsigned int&, unsigned int&, double&, double&, const std::vector<RadialObstacleView>&, double, Angle) throw ();

  /** In einem Zwickel nach dem naechstliegenden Hindernis suchen:
      arg1: RadialObstacleView-Liste in der gesucht werden soll
      arg2: rechte Begrenzungswinkel
      arg3: linker Begrenzungswinkel
      Return: >=100000, wenn kein Hindernis auf dem Strahl liegt, sonst der Abstand zu diesem Hindernis in mm */
  double cone_radial_obstacle_view (const std::vector<RadialObstacleView>&, Angle, Angle) throw (); 

}

#endif
