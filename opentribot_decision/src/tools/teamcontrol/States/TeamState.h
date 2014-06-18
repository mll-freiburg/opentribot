
#ifndef TribotsTools_TeamState_h
#define TribotsTools_TeamState_h

#include "../../../Structures/GameState.h"
#include "../../../Structures/FieldGeometry.h"

namespace TribotsTools {

  /** Datentyp, um die relevanten Informationen fuer das Team zu verwalten; Perspektive Tribots */
  struct TeamState {
    Tribots::RefereeState refstate;             ///< aktueller Refereestate
    int own_half;                               ///< eigene Seite des Tribotsteams (+1=gelb, -1=blau)
    int team_color;                             ///< Farbmarker (+1=cyan, -1=magenta)
    Tribots::FieldGeometry field_geometry;      ///< Feldgeometrie

    std::string refbox_ip;                      ///< Adresse der Refereebox
    int refbox_port;                            ///< Adresse der Refereebox
    bool refbox_connected;                      ///< wurde mit Refereebox verbunden?
    bool refbox_okay;                           ///< ist Kommunikation mit Refereebox okay?

    Tribots::RefboxSignal refbox_signal;        ///< letztes Signal der Refereebox

    bool send_synch_signal;                     ///< Synchronisationssignal senden?
    unsigned short int synch_signal;            ///< Synchronisationssignal
    unsigned int send_lines_rate;               ///< Haeufigkeit, mit der Linien und Hindernisse uebertragen werden
    unsigned int comm_rate;                     ///< Kommunikationstakt in ms

    unsigned int own_score;                     ///< Tore geschossen
    unsigned int opponent_score;                ///< Tore kassiert
    
    int localization_side;                      ///< vom gelben auf das blaue Tor gesehen rechts=+1, links=-1

    TeamState () :
      refstate (Tribots::stopRobot),
      own_half (1),
      team_color (1),
      refbox_ip ("localhost"),
      refbox_port (28097),
      refbox_connected (false),
      refbox_okay (false),
      refbox_signal (Tribots::SIGnop),
      send_synch_signal (false),
      send_lines_rate (1),
      comm_rate (150),
      own_score (0),
      opponent_score (0),
      localization_side (1) {;}

  };

}


#endif
