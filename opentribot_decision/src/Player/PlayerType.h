
#ifndef tribots_player_type_h
#define tribots_player_type_h

#include "../Structures/DriveVector.h"
#include "../Structures/TacticsBoard.h"
#include "../Fundamental/Time.h"
#include <vector>
#include <string>


namespace Tribots {

  /** Abstrakte Klasse als Schnittstelle der Planungs- und Entscheidungskomponente nach innen */
  class PlayerType {
  public:
    virtual ~PlayerType () throw () {;}

    /** berechne einen Fahr- und Kickbefehl
        Arg1: Zeitpunkt, zu dem der Fahrtvektor vsl. gesetzt wird
        Ret: gewuenschter Fahr- und Kickbefehl (Geschwindigkeiten) 
        Seiteneffekte: greift auf die Informationen des Weltmodells zu (sollte nur lesender Zugriff sein) */
    virtual DriveVector process_drive_vector (Time) throw ()=0;

    /** aktuelle Rolle anfragen */
    virtual const char* get_role () throw () =0;
    /** Rolle wechseln, falls moeglich; Arg1: Rollenbeschreibung, Arg2: Erfolg? */
    virtual bool set_role (const char*) throw () =0;
    /** Liste aller Rollen anfragen */
    virtual const std::vector<std::string>& get_list_of_roles () throw () =0;

    /** Taktik durch uebergeben eines neuen Taktikboards veraendern */
    virtual void updateTactics (const TacticsBoard&) throw () =0;
  };

}

#endif
