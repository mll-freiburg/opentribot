
#ifndef Tribots_LocationShortTermMemory
#define Tribots_LocationShortTermMemory

#include "../../Fundamental/RingBuffer.h"
#include "../../Fundamental/Time.h"
#include "../../Structures/RobotLocation.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/ObstacleLocation.h"
#include "../Types/WorldModelTypeBase.h"
#include "ObjectInteractionManager.h"

namespace Tribots {

  class LocationShortTermMemory {
  public:
    /** Konstruktor, uebergeben wird eine Referent auf das eigentliche Weltmodell,
	aus dem ggf. Positionen geholt werden koennen, falls nicht bereits vorhanden */
    LocationShortTermMemory (const WorldModelTypeBase&) throw (std::bad_alloc);
    
    /** hole Roboterposition zum Zeitpunkt arg1, arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    const RobotLocation& get_robot_location (Time, bool) throw ();
    /** hole Ballposition zum Zeitpunkt arg1, arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    const BallLocation& get_ball_location (Time, bool) throw ();
    /** hole Hindernisposition zum Zeitpunkt arg1, arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    const ObstacleLocation& get_obstacle_location (Time, bool) throw ();
    /** Puffer loeschen */
    void clear() throw ();

  private:
    const WorldModelTypeBase& wm;
    ObjectInteractionManager interaction_manager;

    template<class Loc>
    struct CycleTimeLocationTriple {
      unsigned long int cycle;
      Time timestamp;
      Loc value;
    };

    RingBuffer<CycleTimeLocationTriple<RobotLocation> > robots_pure;  // die gespeicherten Roboterpositionen, ohne Interaktion
    RingBuffer<CycleTimeLocationTriple<RobotLocation> > robots_interacted;  // die gespeicherten Roboterpositionen, mit Interaktion
    RingBuffer<CycleTimeLocationTriple<BallLocation> > balls_pure;  // die gespeicherten Ballpositionen, ohne Interaktion
    RingBuffer<CycleTimeLocationTriple<BallLocation> > balls_interacted;  // die gespeicherten Ballpositionen, ohne Interaktion
    CycleTimeLocationTriple<ObstacleLocation> obstacles;  // da z.Z. noch keine Hindernispraediktion oder -interaktion

    void get_interacted (Time);  // interagierte Locations berechnen und einspeichern
    void obstacle_teammate_assignment ();  // Mitspieler den Hindernissen zuordnen, soweit moeglich
  };

}

#endif
