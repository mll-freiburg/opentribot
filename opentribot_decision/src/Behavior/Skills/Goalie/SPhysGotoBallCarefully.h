
#ifndef Tribots_SPhysGotoBallCarefully_h
#define Tribots_SPhysGotoBallCarefully_h

#include "../../Skill.h"
#include "SPhysGotoPos.h"

namespace Tribots {

  /** Faehrt den Ball aus einer bestimmten Richtung an.
      Faehrt dicht an den Ball ran, ohne ihn zu beruehren.
      haelt bei Vorbeifahrt vor dem Ball einen groesseren Abstand ein.
      Ignoriert Hindernisse. */
  class SPhysGotoBallCarefully : public Skill {
  public:
    /** Konstruktor, liest dynamische Eigeschaften (Geschwindigkeiten, Beschleunigungen) aus den RobotProperties */
    SPhysGotoBallCarefully () throw ();
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw();
    /** Fahrtbefehl erzeugen, arg3=anzufahrende Position (statt tatsaechlicher Ballposition) */
    DriveVector getCmd(const Time&, Vec) throw();
    /** initialisieren mit:
      Arg1: Zielausrichtung
      Arg2: Orientierung der Anfahrt: +1 im Gegenuhrzeigersinn (Linkskurve), -1 im Uhrzeigersinnn (Rechtskurve)
      Arg3: hinter dem Ball warten? */
    void init (Angle, int, bool =false) throw ();
    /** initialisieren mit:
      Arg1: Zielausrichtung (=Durchfahrtsrichtung)
      Arg2: Orientierung der Anfahrt: +1 im Gegenuhrzeigersinn (Linkskurve), -1 im Uhrzeigersinnn (Rechtskurve)
      Arg3: hinter dem Ball warten? */
    void init (Vec, int, bool =false) throw ();
    /** initialisieren mit:
      Arg1: Zielausrichtung
      Arg2: hinter dem Ball warten?
      Anfahrtrichtung wird so bestim0.5*fgeom.ball_diameter+MWM.get_robot_properties().kicker_distance+100mt, dass der Weg des Anfahrtweges minimal wird */
    void init (Angle, bool =false) throw ();
    /** initialisieren mit:
      Arg1: Zielausrichtung (=Durchfahrtsrichtung)
      Arg2: hinter dem Ball warten?
      Anfahrtrichtung wird so bestim0.5*fgeom.ball_diameter+MWM.get_robot_properties().kicker_distance+100mt, dass der Weg des Anfahrtweges minimal wird */
    void init (Vec, bool =false) throw ();
    /** die translatorische Hoechstgeschwindigkeit setzen */
    void set_dynamics (double) throw ();
    /** die translatorische (arg1) und rotatorische Hoechstgeschwindigkeit setzen */
    void set_dynamics (double, double) throw ();
    /** die dynamischen Eigenschaften setzen: 
      Arg1: maximale translatorische Geschwindigkeit in m/s,
      Arg2: maximale rotationale Geschwindigkeit in rad/s,
      Arg3: maximale translatorische Beschleunigung in m/s^2,
      Arg4: maximale rotationale Bschleunigung in rad/s^2 */
    void set_dynamics (double, double, double, double) throw ();

  private:
    // Zielwerte:
    Angle target_heading;
    int dir;
    bool wait;

    // Geometrie:
    double ball_radius;
    double robot_half_width;
    double robot_half_length;

    SPhysGotoPos goto_pos_skill;
  };

}

#endif 
