
#ifndef Tribots_SPhysGotoBallAvoidObstacles_h
#define Tribots_SPhysGotoBallAvoidObstacles_h

#include "../../Skill.h"
#include "SPhysGotoPosAvoidObstacles.h"
#include "../../../Fundamental/geometry.h"
#include <vector>

namespace Tribots {

  /** Faehrt Ball an. Umfaehrt Hindernisse und den Ball */
  class SPhysGotoBallAvoidObstacles : public Skill {
  public:
    /** Konstruktor, liest dynamische Eigeschaften (Geschwindigkeiten, Beschleunigungen) aus den RobotProperties */
    SPhysGotoBallAvoidObstacles () throw ();
    ~SPhysGotoBallAvoidObstacles () throw ();
    /** einen theoretischen Kontaktpunkt berechnen, an dem Roboter und Ball zusammentreffen koennten */
    static Vec determineContactPoint (const Time&) throw ();
    /** Fahrtbefehl erzeugen */
    DriveVector getCmd(const Time&) throw();
    DriveVector getCmdNonMovingBall (const Time&) throw ();
    DriveVector getCmdInterceptBall (const Time&) throw ();
    DriveVector getCmdFollowBall (const Time&) throw ();
    /** einige Dinge vergessen */
    void loseControl(const Time&) throw(TribotsException);
    /** initialisieren mit:
      Arg1: Zielausrichtung
      Arg2: Zielgeschwindigkeit in m/s */
    void init (Angle, double) throw ();
    /** initialisieren mit:
      Arg1: Zielausrichtung
      Arg2: Zielgeschwindigkeit in m/s
      Arg3: Toleranzwinkel bei Anfahrt */
    void init (Angle, double, Angle) throw ();
    /** initialisieren mit:
      Arg1: Zielausrichtung (=Durchfahrtsrichtung)
      Arg2: Zielgeschwindigkeit in m/s */
    void init (Vec, double) throw ();
    /** initialisieren mit:
      Arg1: Zielausrichtung (=Durchfahrtsrichtung)
      Arg2: Zielgeschwindigkeit in m/s
      Arg3: Toleranzwinkel bei Anfahrt */
    void init (Vec, double, Angle) throw ();
    /** initialisieren mit Zielgeschwindigkeit fuer direkte Anfahrt */
    void init (double) throw ();
    /** die translatorische Hochstgeschwindigkeit setzen */
    void set_dynamics (double) throw ();
    /** die translatorische (arg1) und rotatorische (arg2) Hochstgeschwindigkeit setzen */
    void set_dynamics (double, double) throw ();
    /** die dynamischen Eigenschaften setzen: 
      Arg1: maximale translatorische Geschwindigkeit in m/s,
      Arg2: maximale rotationale Geschwindigkeit in rad/s,
      Arg3: maximale translatorische Beschleunigung in m/s^2,
      Arg4: maximale rotationale Beschleunigung in rad/s^2 */
    void set_dynamics (double, double, double, double) throw ();

  private:
    enum ApproachDir { hinten, vorne, rechts, links };

    Angle target_heading;
    bool approach_directly;
    double target_velocity;
    bool ball_was_rolling;

    Angle heading_tolerance;
    bool gotoBall;
    
    ApproachDir previous_approach_dir;
    bool remember_approach_dir;
    SPhysGotoPosAvoidObstacles goto_pos_skill;
  };

}

#endif 
