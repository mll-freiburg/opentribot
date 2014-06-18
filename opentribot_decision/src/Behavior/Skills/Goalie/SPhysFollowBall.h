
#ifndef _Tribots_SPhysFollowBall_h_
#define _Tribots_SPhysFollowBall_h_

#include "../../Skill.h"
#include "SPhysGotoPosAvoidObstacles.h"
#include "../../../Fundamental/geometry.h"
#include <vector>

namespace Tribots {

  /** Faehrt von hinten Ball an (gesehen in Ballbewegungsrichtung). Umfaehrt Hindernisse und den Ball */
  class SPhysFollowBall : public Skill {
  public:
    /** Konstruktor, liest dynamische Eigeschaften (Geschwindigkeiten, Beschleunigungen) aus den RobotProperties */
    SPhysFollowBall () throw ();
    ~SPhysFollowBall () throw ();

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

    /** Anfahrt */
    DriveVector getCmd(const Time&) throw();

  protected:
    SPhysGotoPosAvoidObstacles goto_pos_skill;
    double maxvrot;
    double maxvtrans;
    Vec aveballvel;
    Time latest_aveballvel;
  };

}

#endif
