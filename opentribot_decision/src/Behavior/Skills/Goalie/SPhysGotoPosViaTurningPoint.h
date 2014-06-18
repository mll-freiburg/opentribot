
#ifndef Tribots_SPhysGotoPosViaTurningPoint_h
#define Tribots_SPhysGotoPosViaTurningPoint_h

#include "SPhysGotoPos.h"

namespace Tribots {

  /** Faehrt um einen Wendepunkt herum zu einer vorgegebenen 
      Position und haelt dort an. Ignoriert Hindernisse */
  class SPhysGotoPosViaTurningPoint : public Skill {
  public:
    /** Konstruktor, liest dynamische Eigeschaften (Geschwindigkeiten, Beschleunigungen) aus den RobotProperties */
    SPhysGotoPosViaTurningPoint () throw ();
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung
      Arg3: Wendemarke
      Arg4: Radius in mm, in dem Roboter um Wendemarke herumfahren soll
      Arg5: soll Roboter am Ziel anhalten?
      Arg6: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg7: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden? */
    void init (Vec, Angle, Vec, double, bool, bool =true, bool =true) throw ();
    /** initialisieren mit:
      Arg1: Zielposition in Weltkoordinaten,
      Arg2: Zielausrichtung (=Durchfahrtsrichtung)
      Arg3: Wendemarke
      Arg4: Radius in mm, in dem Roboter um Wendemarke herumfahren soll
      Arg5: soll Roboter am Ziel anhalten?
      Arg6: sollen kleine Ungenauigkeiten bei der Position toleriert werden?
      Arg7: sollen kleine Ungenauigkeiten bei der Orientierung toleriert werden? */
    void init (Vec, Vec, Vec, double, bool, bool =true, bool =true) throw ();
    /** die translatorische (arg1) und rotatorische (arg2) Hochstgeschwindigkeit setzen */
    void set_dynamics (double, double) throw ();
    /** die dynamischen Eigenschaften setzen: 
      Arg1: maximale translatorische Geschwindigkeit in m/s,
      Arg2: maximale rotationale Geschwindigkeit in rad/s,
      Arg3: maximale translatorische Beschleunigung in m/s^2,
      Arg4: maximale rotationale Beschleunigung in rad/s^2 */
    void set_dynamics (double, double, double, double) throw ();
    /** zwischen zwei Manoevern muss mindestens eines von gainControl oder loseControl aufgerufen werden (nicht notwendiger Weise beide) */
    void gainControl(const Time&) throw();
    /** zwischen zwei Manoevern muss mindestens eines von gainControl oder loseControl aufgerufen werden (nicht notwendiger Weise beide) */
    void loseControl(const Time&) throw();

  private:
    // Zielwerte:
    Vec target_pos;
    Angle target_heading;
    Vec turning_point;
    double turning_radius;
    bool do_stop;
    bool tolerance_pos;
    bool tolerance_heading;

    // interne Werte:
    int phase;  // Bewegungsphase:  0: starte Bewegung, 1: Anfahrt zum Wendepunkt, 2: Umrunden des Wendepunktes, 3: Anfahrt des Ziels
    int dir;  // +1/.1, je nachdem, ob der Wendepunkt links oder rechtsherum umfahren werden soll
    Vec tangent_point1;  // Tangente Start-Wendekreis
    Vec tangent_point2;  // Tangente Wendekreis-Ziel

    SPhysGotoPos goto_pos_skill;
  };

}

#endif 

