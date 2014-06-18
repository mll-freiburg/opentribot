
#ifndef Tribots_GameStateManager_h
#define Tribots_GameStateManager_h

#include "../../Structures/GameState.h"
#include "../../Structures/BallLocation.h"
#include "../../Structures/FieldGeometry.h"
#include "../../Fundamental/ConfigReader.h"
#include "RefereeStateMachine.h"


namespace Tribots {

  /** Klasse, um die Verwaltung der Gamestates */
  class GameStateManager {
  private:
    GameState gs;                ///< aktueller Gamestate
    BallLocation ref_ball;       ///< Referenzposition des Balls
    RefereeStateMachine rsm;     ///< der Uebergangsautomat fuer RefereeStates

  public:
    /** Konstruktor, setzt GameState */
    GameStateManager (const ConfigReader&, const FieldGeometry&) throw ();
    /** Destruktor */
    ~GameStateManager() throw ();
    /** GameState abfragen */
    const GameState& get () const throw ();
    /** Aktualisieren (10-Sekunden-Regeln, Ballbewegung) */
    void update () throw ();
    /** Signal der Refereebox einarbeiten */
    void update (RefboxSignal) throw ();
    /** Ball in das Spiel (true) oder aus dem Spiel (false) nehmen */
    bool get_in_game () throw ();
    void set_team_signal (int signal) throw (){gs.teamsignal=signal;};
    void set_in_game (bool) throw ();
    /** Mitteilung des Beginns eines neuen Zyklus
        Arg1: Startzeitpunkt fuer neuen Zyklus
        Arg2: Erwartete Ausfuehrungszeit fuer Fahrtbefehl in naechstem Zyklus */
    void init_cycle (Time, Time) throw ();
    /** Spielstand und Gelbe Karten setzen */
    void set_score (unsigned int, unsigned int, unsigned int) throw ();
  };

}


#endif
