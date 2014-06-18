#ifndef _Tribots_KickTestPlayer_h_
#define _Tribots_KickTestPlayer_h_

#include "SingleRolePlayer.h"
#include "../Fundamental/ConfigReader.h"
#include "../Behavior/Skills/Goalie/SPhysGotoPosAvoidObstacles.h"
#include "../Behavior/Skills/BallHandling/SDribbleBallToPosRL.h"

namespace Tribots {

    /**
     * Spielertyp, der im Tutorial verwendet wird. Reagiert auf game-stopped.
     */
    class NewKickerTestPlayer : public SingleRolePlayer {
    public:
        /** Konstruktor. Wird bei Instanzierung eines TutorialPlayers aufgerufen.
         */
        NewKickerTestPlayer(const ConfigReader&) throw ();

        /** Destruktor. Wird bei der Zerstˆrung einer Instanz von TutorialPlayer
         *  aufgerufen */
        ~NewKickerTestPlayer() throw () {
        }
		  
		  void updateTactics(const TacticsBoard& tb) throw ();

        /** Berechnet einen Fahrvektor f¸r den Zeitpunkt t */
        DriveVector process_drive_vector(Time t) throw ();
    protected:
        Time last_kick;
		  Time have_ball;
        SPhysGotoPosAvoidObstacles* goto_pos_skill;
        SDribbleBallToPosRL* dribble_skill;
        Vec start_pos;
        double velocity;
        Vec shoot_pos;
		  bool just_kicked;
		  double kick_height;
		  bool debug;
    };

}

#endif

