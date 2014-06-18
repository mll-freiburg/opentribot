
#ifndef _Tribots_Player_h_
#define _Tribots_Player_h_

#include <stdexcept>
#include "PlayerType.h"
#include "../Structures/TribotsException.h"
#include "../Structures/TacticsBoard.h"
#include "../WorldModel/WorldModel.h"
#include "../Fundamental/ConfigReader.h"


namespace Tribots {

  /** Schnittstelle der Planungs- und Entscheidungskomponente nach aussen
      Aufgaben: Berechnen eines Fahrtvektors, Verwaltung verschiedener Spielertypen */
  class Player {
  private:
    PlayerType* the_player;
    char* player_descriptor;
    const ConfigReader& configuration_list;
    TacticsBoard tactics;

    /** eigentliche Implementierung des Spielerwechsels */
    void really_change_player_type (const char*, const ConfigReader&) throw (TribotsException, std::bad_alloc);
  public:
    /** Konstruktor
        Arg1: ConfigReader, aus dem alle relevanten Parameter (Spielertyp, Einsatzbereich, usw.) ausgelesen werden 
        Arg2: Beschreibung der Robotereigenschaften (Geschwindigkeit, Beschleunigung, usw.) */
    Player (const ConfigReader&) throw (TribotsException, std::bad_alloc);
    ~Player () throw ();


    /** Wechsel des Spielertyps
        Spielertyp-Parameter werden aus dem ConfigReader gelesen, der mit dem Konstruktor uebergeben wurde
        Arg1: Bezeichner fuer Spielertyp
        Ret: bei Erfolg, true */
    bool change_player_type (const char*) throw ();
    /** Wechsel des Spielertyps 
        Arg1: Bezeichner fuer Spielertyp 
        Arg2: Parameterliste fuer neuen Spielertyp 
        Ret: bei Erfolg, true */
    bool change_player_type (const char*, const ConfigReader&) throw ();

    /** liefere eine Beschreibung des aktuellen Spielertyps */
    const char* get_player_type () const throw ();

    void getPlayerTypeList(std::vector<std::string> &ptl);

    /** berechne einen Fahr- und Kickbefehl
        Arg1: Zeitpunkt, zu dem der Fahrtvektor vsl. gesetzt wird
        Ret: gewuenschter Fahr- und Kickbefehl (Geschwindigkeiten) 
        Seiteneffekte: greift auf die Informationen des Weltmodells zu (sollte nur lesender Zugriff sein) */
    inline DriveVector process_drive_vector (Time t) throw () { return (the_player->process_drive_vector (t)); }

    /** aktuelle Rolle anfragen */
    inline const char* get_role () throw () { return the_player->get_role(); }
    /** Rolle wechseln, falls moeglich; Arg1: Rollenbeschreibung, Arg2: Erfolg? */
    bool set_role (const char* s) throw ();
    /** Liste aller Rollen anfragen */
    inline const std::vector<std::string>& get_list_of_roles () throw () { return the_player->get_list_of_roles(); }

    /** Taktik durch uebergeben eines neuen Taktikboards veraendern */
    void updateTactics (const TacticsBoard&) throw (std::bad_alloc);
    /** aktuelles Taktikboard anfragen */
    const TacticsBoard& getTactics () const throw ();
  };

}

#endif
