
#ifndef _Tribots_update_robot_location_h
#define _Tribots_update_robot_location_h

#include "../../Structures/RobotLocation.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/ObstacleLocation.h"
#include "../../Structures/DriveVector.h"

namespace Tribots {
 
  /** Funktion zum Fortberechnen von Roboterposition und -Geschwindigkeit durch eine Fahrt
      Angaben in WELTKOORDINATEN (Euler-Verfahren, stufenweise Integration)
      Arg1: Ausgangsposition und -Geschwindigkeit
      Arg2: Zeitdauer der Bewegung in msec
      Ret: Position und Geschwindigkeit nach Ausfuehren der Bewegung */
  RobotLocation update_robot_location (const RobotLocation&, double) throw ();

  /** Funktion zum Fortberechnen von Roboterposition und -Geschwindigkeit durch eine Fahrt
    (Heun-Verfahren, Trapezregel)
    Arg1: Ausgangsposition und -Geschwindigkeit (Weltkoordinaten)
    Arg2: Zeitdauer der Bewegung in msec
    Arg3: Robotergeschwindigkeit zu Zielzeitpunkt (Roboterkoordinaten)
    Ret: Position und Geschwindigkeit nach Ausfuehren der Bewegung (Weltkoordinaten) */
  RobotLocation update_robot_location (const RobotLocation&, double, const DriveVector&) throw ();

  /** Funktion zum Spiegeln eines RobotLocation Objektes in arg1;
      arg2 gibt an, ob gespiegelt werden soll (-1) oder nicht (+1). In letzterem
      Fall ist return = arg1 */      
  RobotLocation flip_sides (const RobotLocation&, int =-1) throw ();

  /** Funktion zum Spiegeln eines BallLocation Objektes in arg1;
      arg2 gibt an, ob gespiegelt werden soll (-1) oder nicht (+1). In letzterem
      Fall ist return = arg1 */      
  BallLocation flip_sides (const BallLocation&, int =-1) throw ();

  /** Funktion zum Spiegeln eines ObstacleLocation Objektes in arg1;
      arg2 gibt an, ob gespiegelt werden soll (-1) oder nicht (+1). In letzterem
      Fall ist return = arg1 */      
  ObstacleLocation flip_sides (const ObstacleLocation&, int =-1) throw ();

}

#endif

