
#ifndef Tribots_ObstacleContainer_h
#define Tribots_ObstacleContainer_h

#include "../../Structures/VisibleObject.h"
#include "../../Structures/ObstacleLocation.h"
#include "../../Structures/RobotLocation.h"
#include "../../Structures/FieldGeometry.h"
#include "../../Fundamental/ConfigReader.h"


namespace Tribots {

  /** Hindernis-Container ohne weitere Funktionalitaet
      liefert genau die Hindernisse, die im Bild gesehen wurden */
  class ObstacleContainer {
  private:
    ObstacleLocation obstacles;
    ObstacleLocation poles;
    unsigned int stuck_obstacle_delay;
  public:
    /** Konstruktor */
    ObstacleContainer (const ConfigReader&, const FieldGeometry&) throw ();

    /** liefere Position der Hindernisse*/
    ObstacleLocation get () const throw ();
    /** liefere Position der Hindernisse einschliesslich der Eckpfosten*/
    ObstacleLocation get_with_poles () const throw ();
    /** analysiere die Hindernisse anhand der visuellen Information;
        uebergeben wird die Liste erkannter Hindernisse und die Roboterposition zum Zeitpunkt
        der Bildinformation */
    /** liefere Position der Hindernisse einschliesslich der Eckpfosten 
        und der Positionen, an denen der Roboter blockiert war */
    ObstacleLocation get_with_poles_and_stuck () const throw ();
    void update (const VisibleObjectList&, const RobotLocation&) throw ();
    /** setze die Hindernisse neu;
        arg1: Positionen der Hindernisse
        arg2: zugehoerige Breiten
        arg3: Zeitpunkt der Informationen */
    void update (const std::vector<Vec>&, const std::vector<double>&, Time) throw ();
  };

}

#endif
