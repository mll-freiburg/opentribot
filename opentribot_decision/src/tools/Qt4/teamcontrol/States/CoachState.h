
#ifndef TribotsTools_CoachState_h
#define TribotsTools_CoachState_h

#include <string>
#include <vector>
#include "../../../../Structures/TacticsBoardDescription.h"

namespace TribotsTools {

  /** Datentyp, um die relevanten Informationen fuer den Coach zu verwalten (nur Daten, die oeffentlich sind) */
  struct CoachState {
    std::string policy_name;                    ///< (Rollenwechsel-) Strategie
    bool ball_position_mode;                    ///< Ballposition kommunizieren?
    bool ball_posession_mode;                   ///< Ballbesitz kommunizieren?
    bool broadcast_mode;                        ///< Alles andere weiter-kommunizieren?
    bool teammate_mode;                         ///< Mitspielerpositionen kommunizieren?
    bool tactics_mode;                          ///< Taktische Varianten uebermitteln?
    std::string extra_message;                  ///< eine ggf. an alle zu sendende zusaetzliche Nachricht
    bool sl_mirror_hint_mode;                   ///< SL-Sitenhinweise generieren?

    std::vector<std::string> list_policies;     ///< Liste aller Strategien
    Tribots::TacticsBoard tactics_board;        ///< Taktische Einstellungen
    std::vector<Tribots::TacticsAttribute> tactics_attributes;   ///< Liste aller Taktikattribute

    CoachState () :
      policy_name ("---"),
      ball_position_mode (false),
      ball_posession_mode (false),
      broadcast_mode (false),
      teammate_mode (false),
      tactics_mode (true),
      sl_mirror_hint_mode (true),
      list_policies (0),
      tactics_attributes (0) {;}

  };

}


#endif
