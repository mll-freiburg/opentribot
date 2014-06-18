
#ifndef Tribots_ObjectInteractionManager_h
#define Tribots_ObjectInteractionManager_h

#include "../../Structures/RobotLocation.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/ObstacleLocation.h"

namespace Tribots {

  class ObjectInteractionManager {
  public:
    /** Objektinterkationen berechnen und Objektlisten zurueckgeben.
        Arg1: (input+return) die Roboterposition zum spaeteren Zeitpunkt
        Arg2: (input+return) die Ballposition zum spaeteren Zeitpunkt
        Arg3: (input+return) die resultierenden Hindernispositionen zum spaeteren Zeitpunkt
        Arg4: die Roboterpsoition zum frueheren Zeitpunkt
        Arg5: die Ballposition zum frueheren Zeitpunkt
        Arg6: die Zeitspanne in ms zwischen frueherem und spaeterem Zeitpunkt
        Beruecksichtigt folgende Objektinteraktionen:
        (a) Roboter bleibt an Hindernis haengen
        (b) Ball bleibt an Hindernis haengen
        (c) Ball bleibt am Roboter haengen
        (d) Roboter klemmt Ball gegen Hindernis */
    void get (RobotLocation&, BallLocation&, ObstacleLocation&, const RobotLocation&, const BallLocation&, unsigned int) const throw (std::bad_alloc);
  };

}

#endif
