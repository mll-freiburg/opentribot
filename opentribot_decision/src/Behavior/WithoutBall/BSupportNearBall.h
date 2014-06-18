#ifndef _BSupportNearBall_H_
#define _BSupportNearBall_H_

#include "../../Skills/BasicMovements/SGoToPosEvadeObstacles.h"
#include "../../Skills/BasicMovements/SBoostToPos.h"
#include "../../Behavior.h"
#include "../../../Fundamental/geometry.h"

namespace Tribots
{
  /**
   * BSupportNearBall wird bei Ballbesitz bei 4 oder mehr Feldspielern bei demjenigen
   * der beiden Stuermer, der nicht den Ball fuehrt, aktiviert. Ist der Ball in der Mitte,
   * nimmt der unterstuetzende Stuermer eine Position seitlich hinter dem Ballfuehrer ein.
   * Wird der Ball in der linken oder rechten Aussenbahn nach vorne gedribbelt, nimmt der
   * unterstuetzende Stuermer eine Position zwischen Strafraumgrenze und Mittellinie
   * in der Verlaengerung des kurzen Pfostens ein.
   */
  class BSupportNearBall : public Behavior
  {
  public:
    BSupportNearBall(const std::string = "BBSupportNearBall");
    virtual ~BSupportNearBall() throw ();

    virtual DriveVector getCmd(const Time&) throw(TribotsException);

    virtual bool checkInvocationCondition(const Time&) throw();
    virtual bool checkCommitmentCondition(const Time&) throw();

    virtual void gainControl(const Time&) throw(TribotsException);
    virtual void loseControl(const Time&) throw(TribotsException);

    virtual void updateTactics (const TacticsBoard&) throw ();

  protected:
    SGoToPosEvadeObstacles* skill; ///< skill zur Positionsanfahrt mit Hindernisausweichen
    SBoostToPos*       fast_skill; ///< skill zur schnellen Positionsanfahrt, keine Feinregelung
    Skill*             activeSkill; ///< aktuell verwendeter Skill

    enum { BLOCK_LEFT=0, BLOCK_RIGHT };
    enum { BALL_LEFT, BALL_MIDDLE, BALL_RIGHT};
    int ballInLane;          ///< gibt an, in welcher der drei Angriffsspuren sich der Ball befindet. Umschalten mit Hysterese.
    int blockPosition;       ///< gibt an, ob dieser Spieler auf der linken oder rechten Seite hinter dem Ball unterstuetzt, wenn der Ball in der Mittelspur gefuehrt wird.
    float transVel;          ///< Fahrtgeschwindigkeit, kann ueber das Taktikboard ("AllgemeineFahrgeschwindigkeit") angepasst werden.
    
    enum { TACTICS_DEFENSIVE, TACTICS_NEUTRAL, TACTICS_OFFENSIVE };
    int presentTactics;
    bool boostAllowed;
  };

}
#endif
