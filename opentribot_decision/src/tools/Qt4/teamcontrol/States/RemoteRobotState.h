
#ifndef TribotsTools_RemoteRobotState_h
#define TribotsTools_RemoteRobotState_h

#include "../../../../Structures/RobotLocation.h"
#include "../../../../Structures/TeammateLocation.h"
#include "../../../../Structures/BallLocation.h"
#include "../../../../Structures/GameState.h"
#include "../../../../Structures/DriveVector.h"
#include "../../../../Structures/RobotData.h"
#include "../../../../Structures/MessageBoard.h"
#include "../../../../Structures/ObstacleLocation.h"
#include "../../../../Structures/VisibleObject.h"
#include "../../../../Structures/FieldGeometry.h"
#include "../../../../Structures/TacticsBoard.h"
#include "../../../../Communication/UDPCommunicationStatistics.h"
#include <vector>


namespace TribotsTools {

  /** Datentyp, um die relevanten Informationen fuer einen Roboter zu verwalten */
  struct RemoteRobotState {
    std::string name;                           ///< Name des Roboters
    std::string ip;                             ///< IP-Adresse
    int port;                                   ///< Port fuer Kommunikation
    unsigned int id;                            ///< Nummer des Roboters (die am Laetzchen angeschriebene)
    unsigned int local_id;                      ///< dito, jedoch das, was der Roboter selbst annimmt, wer er sei

    bool comm_started;                          ///< Kommunikation initiiert?
    bool comm_okay;                             ///< Kommunikation okay?
    bool comm_interrupted;                      ///< Kommunikation fuer mehrere Zyklen gestoert?
    Tribots::UDPCommunicationStatistics comm_statistics_send; ///< Kommunikationsstatistik fuer diesen Roboter im Ausgang
    Tribots::UDPCommunicationStatistics comm_statistics_receive; ///< Kommunikationsstatistik fuer diesen Roboter im Eingang

    Tribots::RobotLocation robot_pos;           ///< Position des Roboters
    Tribots::TeammateOccupancyGrid occ_grid;    ///< OccupancyGrid des Roboters
    Tribots::BallLocation ball_pos;             ///< Position des Balls
    Tribots::ObstacleLocation obs_pos;          ///< Hindernisse
    Tribots::RefereeState refstate;             ///< RefereeState des Roboters
    bool in_game;                               ///< Roboter aktiviert?
    int own_half;                               ///< eigene Haelfte (+1/-1)
    Tribots::RobotData robot_data;              ///< die Roboterdaten
    std::vector<Tribots::VisibleObjectList> visible_objects; ///< die gesehenen Objekte
    unsigned int own_score;                     ///< Spielstand: eigene Tore
    unsigned int opponent_score;                ///< Spielstand: gegnerische Tore
    unsigned int yellow_cards;                  ///< Anzahl gelbe Karten, die der Roboter glaubt zu haben
    unsigned int desired_yellow_cards;          ///< Anzahl gelbe Karten, die der Roboter haben sollte

    std::string playertype;                     ///< Spielertyp
    std::string playerrole;                     ///< Spielerrolle
    std::vector<std::string> list_players;      ///< Liste moeglicher Spielertypen
    std::vector<std::string> list_roles;        ///< Liste moeglicher Rollen

    Tribots::MessageBoard message_board;        ///< Message-Board
    Tribots::TacticsBoard tactics_board;        ///< Taktik-Einstellungen

    bool desired_connect;                       ///< soll verbunden werden?
    bool desired_in_game;                       ///< in was soll in_game geaendert werden?
    int desired_own_half;                       ///< in was soll own_half geaendert werden?
    Tribots::RefereeState desired_refstate;     ///< in was soll refstate geaendert werden?
    std::string desired_playertype;             ///< in was soll Playertype geaendert werden?
    std::string desired_playerrole;             ///< in was soll Playerrole geaendert werden?

    bool robot_data_request;                    ///< sollen Roboterdaten uebertragen werden?
    bool obstacle_request;                      ///< sollen Hindernisse uebertragen werden?
    bool visible_object_request;                ///< sollen gesehenen Objekte uebertragen werden?
    bool joystick_request;                      ///< Joystick aktiviert?
    bool slhint_request;                        ///< soll SL-Hint uebertragen werden?
    bool debug_image_request;                   ///< Debug-Image erzeugen?
    bool sl_mirror_hint_request;                ///< SL-Mirror-Hint uebertragen?
    bool drive_to_request;                      ///< soll an Zielposition gefahren werden?
  
    Tribots::Vec slhint_pos;                    ///< Werte fuer den SL-Hint
    Tribots::Angle slhint_angle;                ///< Werte fuer den SL-Hint
    Tribots::Vec sl_mirror_hint;                ///< Werte fuer SL-Mirror Hint
    Tribots::Vec drive_to_pos;                  ///< Zielwert fuer drive_to-Funktion
    Tribots::Angle drive_to_angle;              ///< Zielwert fuer drive_to-Funktion

    bool show_message_board;                    ///< MessageBoard auf Konsole anzeigen?

    Tribots::FieldGeometry field_geometry;      ///< die Feldgeometrie, zu Kontrollzwecken

    RemoteRobotState () :
      name ("unknown"),
      ip ("localhost"),
      port (6012),
      id (0),
      local_id (0),
      comm_started (false),
      comm_okay (false),
      comm_interrupted (true),
      refstate (Tribots::stopRobot),
      in_game (false),
      own_half (1),
      own_score (0),
      opponent_score (0),
      yellow_cards (0),
      desired_yellow_cards (0),
      playertype ("unknown"),
      playerrole ("unknown"),
      desired_connect (false),
      desired_in_game (false),
      desired_own_half (1),
      desired_refstate (Tribots::stopRobot),
      desired_playertype ("unknown"),
      desired_playerrole ("unknown"),
      robot_data_request (false),
      obstacle_request (false),
      visible_object_request (false),
      joystick_request (false),
      slhint_request (false),
      debug_image_request (false),
      sl_mirror_hint_request (false),
      drive_to_request (false),
      show_message_board (false) {;}

  };
  
}


#endif
