
#ifndef _Tribots_SPhysVolley_h_
#define _Tribots_SPhysVolley_h_

#include "../../Skill.h"
#include "SPhysGotoPosAvoidObstacles.h"
#include "../../../Fundamental/geometry.h"
#include <vector>

namespace Tribots {

  /** Faehrt Ball an. Umfaehrt Hindernisse und den Ball */
  class SPhysVolley : public Skill {
  public:
    /** Konstruktor, liest dynamische Eigeschaften (Geschwindigkeiten, Beschleunigungen) aus den RobotProperties */
    SPhysVolley () throw ();
    ~SPhysVolley () throw ();

    /** die translatorische (arg1) Hoechstgeschwindigkeit setzen */
    void set_dynamics (double) throw ();
    /** die translatorische (arg1) und rotatorische (arg2) Hoechstgeschwindigkeit setzen */
    void set_dynamics (double, double) throw ();
    /** die dynamischen Eigenschaften setzen:
    Arg1: maximale translatorische Geschwindigkeit in m/s,
    Arg2: maximale rotationale Geschwindigkeit in rad/s,
    Arg3: maximale translatorische Beschleunigung in m/s^2,
    Arg4: maximale rotationale Beschleunigung in rad/s^2 */
    void set_dynamics (double, double, double, double) throw ();
    
    /** Zielausrichtungspunkt (arg1) und Kontaktgeschwidnigkeit (arg2) festlegen */
    void init (Vec, double =3) throw ();

    /** Anfrage nach dem Punkt, an den der Roboter fahren wuerde, wenn getCmd() aufgerufen werden wuerde */
    Vec getInterceptPoint (const Time&) throw ();
    /** Anfahrt */
    DriveVector getCmd(const Time&) throw();

  protected:
    Vec target_directed;     ///< Zielpunkt, auf den der Roboter bei Ballkontakt gerichtet sein soll
    double target_velocity;   ///< maximale Geschwindigkeit bei Roboter-Ball-Kontakt
    
    SPhysGotoPosAvoidObstacles goto_pos_skill;
  };

}

#endif 
