
#ifndef Tribots_SPhysTurnAroundPos_h
#define Tribots_SPhysTurnAroundPos_h

#include "../../Skill.h"
#include "SPhysGotoPos.h"

namespace Tribots {

  /** Umfaehrt eine Position, um in eine bestimmte Ausrichtung zu gelangen.
      Ignoriert Hindernisse. */
  class SPhysTurnAroundPos : public Skill {
  public:
    /** Konstruktor, liest dynamische Eigeschaften (Geschwindigkeiten, Beschleunigungen) aus den RobotProperties */
    SPhysTurnAroundPos () throw ();
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw();
    /** initialisieren mit: Arg1: Mittelpunkt der Bewegung, Arg2: Zielausrichtung, Arg3: Radius */
    void init (Vec, Angle, double) throw ();
    /** initialisieren mit: Arg1: Mittelpunkt der Bewegung, Arg2: Zielausrichtung (=Durchfahrtsrichtung), Arg3: Radius */
    void init (Vec, Vec, double) throw ();
    /** die translatorische (arg1) und rotatorische (arg2) Hochstgeschwindigkeit setzen */
    void set_dynamics (double, double) throw ();
    /** die dynamischen Eigenschaften setzen: 
      Arg1: maximale translatorische Geschwindigkeit in m/s,
      Arg2: maximale rotationale Geschwindigkeit in rad/s,
      Arg3: maximale rotationale Beschleunigung in rad/s^2
      Arg4: maximale translatorische Beschleunigung in m/s^2 */
    void set_dynamics (double, double, double, double) throw ();

  private:
    // Zielwerte:
    Angle target_heading;
    double target_radius;
    Vec center;

    SPhysGotoPos goto_pos_skill;
  };

}

#endif 
