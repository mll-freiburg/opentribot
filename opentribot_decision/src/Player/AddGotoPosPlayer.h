
#ifndef _Tribots_AddGotoPosPlayer_h_
#define _Tribots_AddGotoPosPlayer_h_

#include "PlayerType.h"
#include "../Fundamental/ConfigReader.h"
#include "../Behavior/Skills/Goalie/SPhysGotoPos.h"
#include "../Structures/GameState.h"

namespace Tribots {

  /** Klasse AddGotoPosPlayer als zusaetzlicher Player zu einer "normalen"
      Playerklasse. D.h., wird ueber das MessageBoard ein GotoPos-Signal
      gesendet, so uebernimmt dieser Spieler die Kontrolle und faehrt zur
      angegebenen Position. Anschliessend wartet er bis zum naechsten
      Wechsel des Referee-States */
  class AddGotoPosPlayer : public PlayerType {
  private:
    PlayerType* the_elementary_player;             // die "normale" Playerklasse
    bool is_active;                                // GotoPos aktiv? (oder richtiger Spieler?)
    bool arrived;                                  // an Zielposition angekommen?
    RefereeState latest_refstate;                  // letzter Refereestate
    SPhysGotoPos goto_pos_skill;                   // Skill, um zu einer Position zu fahren
    Vec target_pos;                           // Zielposition
    Angle target_heading;                // Zielausrichtung
  public:
    /** Initialisierung wie bei GotoPosPlayer; arg2 ist Zeiger auf "normale" Playerklasse */
    AddGotoPosPlayer (const ConfigReader&, PlayerType*) throw (InvalidConfigurationException, std::bad_alloc);
    /** Destruktor */
    ~AddGotoPosPlayer () throw ();

    /** Berechnung eines DriveVector */
    DriveVector process_drive_vector (Time) throw ();

    void updateTactics (const TacticsBoard&) throw ();
    const char* get_role () throw ();
    bool set_role (const char*) throw ();
    const std::vector<std::string>& get_list_of_roles () throw ();
  };

}

#endif

