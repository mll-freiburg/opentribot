
#ifndef Tribots_RefereeStateMachine_h
#define Tribots_RefereeStateMachine_h

#include "../../Structures/GameState.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/ObstacleLocation.h"
#include "../../Structures/FieldGeometry.h"
#include "../../Fundamental/Time.h"


namespace Tribots {

  /** realisiert den Uebergangsgraphen fuer Refereestates */
  class RefereeStateMachine {
  private:
    int team;                // +1 = cyan, -1=magenta
    RefereeState current;    // aktueller Zustand
    Time latest_update;      // letzte Aktualisierung
    Vec ballpos;             // Ballreferenzposition
    bool ballknown;          // Ballposition bekannt?
    double penalty_marker_y; // y-Position des Penalty-Markers
    double center_circle_radius;  // Mittelkreisradius
    double precision_addon;  // zusaetzlicher Sicherheitsabstand bei Ballbewegung, im Fall dass der Roboter faehrt, wenn refstate wechselt
  public:
    /** Konstruktor;
        Arg1: Teamfarbe +1 cyan oder -1 magenta
        Arg2: aktueller Zustand
        Arg3: Geometrie des Feldes */
    RefereeStateMachine (const FieldGeometry&, int =1, RefereeState =stopRobot) throw ();
    /** Copy-Konstruktor */
    RefereeStateMachine (const RefereeStateMachine&) throw ();
    /** Zuweisungsoperator */
    const RefereeStateMachine& operator= (const RefereeStateMachine&) throw ();

    /** RefereeState abfragen */
    RefereeState get_state () const throw ();

    /** RefereeState setzen */
    void set_state (RefereeState) throw ();
    /** Teamfarbe setzen; +1=cyan, -1=magenta */
    void set_team_color (int) throw ();

    /** Signal einarbeiten;
        Arg1: Signal
        Arg2: Ballposition
        Arg3: Roboterposition
        Arg4: Robotergeschwindigkeit
        Arg5: Roboter-Winkelgeschwindigkeit
        Return: das Signal in own/opponent-Codierung */
    RefboxSignal update (RefboxSignal, const BallLocation&, Vec, Vec, double) throw ();
    /** zyklische Aktualisierung wegen 10s-Regel; muss regelmaessig aufgerufen werden;
        Arg1: Ballposition
        Arg2: Roboterposition
        Arg3: Roboterspeed
        Arg4: Hindernisliste */
    void update (const BallLocation&, Vec, Vec, const ObstacleLocation&) throw ();
  };

}

#endif
