
#ifndef _Tribots_TacticsBoardDescription_h_
#define _Tribots_TacticsBoardDescription_h_

#include "TacticsBoard.h"
#include "../Fundamental/ConfigReader.h"
#include <vector>

namespace Tribots {

  /** Struktur, um ein Taktikattribut zu beschreiben */
  struct TacticsAttribute {
    std::string name;                           ///< Name des Attributs
    std::vector<std::string> options;           ///< Liste aller moeglichen Werte
    std::string default_value;                  ///< Default-Wert
    bool editable;                              ///< Attribut erweiterbar durch Eingabe
    bool force_default;                         ///< Soll Annahme dieses Wertes im Teamcontrol erzwungen werden?
  };

  /** Funktion, um aus einem ConfigReader (arg2) eine Beschreibung der Taktikvarianten (arg1, return) zu berechnen */
  void read_tactics (std::vector<TacticsAttribute>&, const ConfigReader&) throw (std::bad_alloc);

  /** Funktion, um ein TaktikBoard mit Default-Eintraegen aus einer Taktikbeschreibung zu berechnen:
      Arg1: (Return) erzeugtes Taktikboard,
      Arg2: (Argument) Taktikbeschreibung,
      Arg3: (Argument) sollen das erzeugte Taktikboard zunaechst geloescht werden (true) oder die vorhanden Eintraege ergaenzt werden (false) ? */
  void make_default_tactics_board (TacticsBoard&, std::vector<TacticsAttribute>&, bool =true) throw (std::bad_alloc);

}

#endif
