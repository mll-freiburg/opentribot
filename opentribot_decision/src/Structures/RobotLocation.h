
#ifndef tribots_robot_location_h
#define tribots_robot_location_h

#include "../Fundamental/Angle.h"
#include "../Fundamental/Vec.h"

namespace Tribots {

  /** Struktur, um Blockadesituationen zu beschreiben; alle Angaben in WELTKOORDINATEN */
  struct RobotStuckLocation {
    bool robot_stuck;                ///< Roboter blockiert?
    unsigned int msec_since_stuck;   ///< Zeit in msec seit letztem Mal, als eine Blockierung festgestellt wurde
    Vec pos_of_stuck;                ///< Position, an der sich der Roboter bei letzter Blockierung befunden hat
    Vec dir_of_stuck;                ///< Richtung, in die der Roboter bei letzter Blockierung fahren wollte
    inline bool operator() () const throw () { return robot_stuck; }  ///< Abfrage des "robot_stuck"-Attributs

    RobotStuckLocation () : robot_stuck(false), msec_since_stuck(100000) {;}
    RobotStuckLocation (const RobotStuckLocation& rs) { operator= (rs); }
    const RobotStuckLocation& operator= (const RobotStuckLocation& rs) {
      robot_stuck=rs.robot_stuck;
      msec_since_stuck=rs.msec_since_stuck;
      pos_of_stuck=rs.pos_of_stuck;
      dir_of_stuck=rs.dir_of_stuck;
      return *this;
    }
  };


  /** Struktur, um die derzeitige Roboterposition auf dem Spielfeld zu repraesentieren 
      alle Koordinaten (in WELTKOORDINATEN) sind relativ zur aktuellen Spielrichtung:
      (0,0) ist der Mittelpunkt des Feldes
      die y-Achse zeigt in Richtung des gegnerischen Tores
      die x-Achse zeigt rechtwinklig dazu (rechtshaendisches Koordinatensystem)
      ein Winkel von 0 bezeichnet die Parallele zur x-Achse
      alle Laengenangaben in mm, alle Winkelangaben in rad 
      alle Geschwindigkeiten in m/s bzw. rad/s */
  struct RobotLocation {
    Vec pos;                  ///< Roboterposition
    Angle heading;            ///< Verdrehung des Roboters, 0=Roboter ist in Richtung gegnerisches Tor ausgerichtet
    Vec vtrans;               ///< translatorische Robotergeschwindigkeit in m/s
    double vrot;              ///< rotative Geschwindigkeit des des Roboters in rad/s
    bool kick;                ///< Kicker aktiv?

    bool valid;               ///< Ist die SL okay?

    RobotStuckLocation stuck; ///< Information, ob und wie Roboter blockiert ist

    RobotLocation () : pos(0,0), heading(0), vtrans(0,0), vrot(0), kick(false), valid(true) {;}
    RobotLocation (const RobotLocation& rl) { operator= (rl); }
    const RobotLocation& operator= (const RobotLocation& rl) {
      pos=rl.pos;
      heading=rl.heading;
      vtrans=rl.vtrans;
      vrot=rl.vrot;
      kick=rl.kick;
      valid=rl.valid;
      stuck=rl.stuck;
      return *this;
    }
  };

}

#endif

