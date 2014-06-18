
#ifndef _Tribots_WorldModelType_h_
#define _Tribots_WorldModelType_h_

#include "../Structures/FieldGeometry.h"
#include "../Structures/VisibleObject.h"
#include "../Structures/DriveVector.h"
#include "../Structures/RobotLocation.h"
#include "../Structures/TeammateLocation.h"
#include "../Structures/BallLocation.h"
#include "../Structures/ObstacleLocation.h"
#include "../Structures/RobotProperties.h"
#include "../Structures/RobotData.h"
#include "../Structures/GameState.h"
#include "../Structures/MessageBoard.h"
#include "../Structures/GyroData.h"
#include <vector>


namespace Tribots {

  class FataMorgana;

  /** Struktur, um den Ball (aus dem Bild) relativ zum Roboter zu repraesentieren */
  struct BallRelative {
    bool valid;   ///< Werte gueltig, d.h. wurde im Bild tatsaechlich der Ball gesehen
    Vec pos;   ///< Position des Balls relativ zur Roboterposition und -ausrichtung, ungefiltert
  };

  /** abstrakte Klasse als Schnittstelle des Weltmodells nach innen */
  class WorldModelType {
  public:
    virtual ~WorldModelType () throw () {;}

    // Informationsgewinnung:
    /** liefere Spielfeldgeometrie */
    virtual const FieldGeometry& get_field_geometry () const throw () =0;
    /** liefere Orientierung des Spielfeldes (eigene Haelfte), +1, wenn Spiel von gelbem Tor auf blaues Tor, sonst -1 */
    virtual int get_own_half () const throw () =0;
    /** liefere letzten Spielzustand */
    virtual const GameState&  get_game_state () const throw () =0;
    /** liefere Roboterposition, Geschwindigkeit und Orientierung zum Zeitpunkt arg1, arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    virtual const RobotLocation& get_robot_location (Time, bool) throw () =0;
    /** liefere Ballposition und Geschwindigkeit zum Zeitpunkt arg1, arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    virtual const BallLocation& get_ball_location (Time, bool) throw () =0;
    /** die relative, ungefilterte Ballposition anfragen */
    virtual const BallRelative& get_ball_relative () throw () =0;
    /** liefere Position der Hindernisse zum Zeitpunkt arg1, arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    virtual const ObstacleLocation& get_obstacle_location (Time, bool) throw () =0;
    /** die Positionen der Mitspieler liefern (sofern bekannt) */
    virtual const std::vector<TeammateLocation>& get_teammate_location () const throw () =0;
    /** Die Roboter-ID anfragen (Laetzchen-nummer) */
    virtual unsigned int get_robot_id () throw () =0;
    /** liefere Robotereigenschaften */
    virtual const RobotProperties& get_robot_properties () const throw () =0;
    /** liefere zusaetzliche Daten des Robters
        Arg1: Referenz auf Zeitobjekt in das die Zeit geschrieben wird von dem die Daten stammen */
    virtual const RobotData& get_robot_data (Time&) const throw () =0;
    /** liefere eine aus der Selbstlokalisierung geschaetzte Geschwindigkeit 
        Arg1: Referenz auf Zeitobjekt in das die Zeit geschrieben wird von dem die Daten stammen */
    virtual const RobotLocation& get_slfilter_robot_location (Time&) const throw () =0;
    /** liefere den zuletzt gesetzten Fahrtvektor */
    virtual const DriveVector& get_recent_drive_vector () const throw () =0;
    /** liefert das zuletzt aktive Behavior, wenn bekannt */
    virtual const char* get_active_behavior () const throw () =0;

    // Informationen einfliessen lassen:
    /** Spielrichtung setzen; Arg1: +1, wenn Spiel von gelbem Tor auf blaues Tor, sonst -1 */
    virtual void set_own_half (int) throw () =0;
    /** Signal der Refereebox einarbeiten */
    virtual void update_refbox (RefboxSignal) throw () =0;
    /** Spielstand (arg1 eigene, arg2 gegnerische Tore) und gelbe Karten (arg3) setzen */
    virtual void set_score (unsigned int, unsigned int, unsigned int) throw () =0;
    /** Roboter starten (true)/stoppen(false), veraendern des "in_game"-Flags */
    virtual void startstop (bool) throw () =0;
    /** neuen Fahr-/Kickvektor mitteilen
        Arg1: Fahrvektor
        Arg2: Zeitpunkt, zu dem der Fahrvektor gesetzt wurde */
    virtual void set_drive_vector (DriveVector, Time) throw () =0;
    /** gemessene Bewegung mitteilen
        Arg1: gemessener Fahrvektor
        Arg2: Zeitpunkt der Messung */
    virtual void set_odometry (DriveVector, Time) throw () =0;
    /** gemessene Rotation mitteilen
        Arg1: gemessene Rotation
        Arg2: Zeitpunkt der Messung */
    virtual void set_gyro_data (GyroData, Time) throw () =0;
    /** neue Information des visuellen Sensors integrieren
        Arg1: Einzelnes Objekt
        Arg2: Zeitstempel fuer dieses Objekt (Zeitpunkt der Aufnahme)
        Arg3: Bildquellen-ID */
    virtual void set_visual_information (const VisibleObject&, Time, unsigned int) throw () =0;
    /** trage Robotereigenschaften ein */
    virtual void set_robot_properties (const RobotProperties&) throw () =0;
    /** neue Roboterdaten setzen, diese werden im Moment nicht weitergerechnet 
        Arg1: Roboterdaten
        Arg2: Zeitpunkt von dem die Daten stammen */
    virtual void set_robot_data (const RobotData&, Time) throw() =0;
    /** neu uebermittelte Positionen der Mitspieler einbinden */
    virtual void set_teammates (std::vector<TeammateLocation>&) throw () =0;
    /** Roboter-ID setzen */
    virtual void set_robot_id (unsigned int) throw () =0;
    /** setze das zuletzt aktive Behavior, wenn anwendbar */
    virtual void set_active_behavior (const char*) throw () =0;
    /** teile den Spielertyp mit */
    virtual void set_player_type (const char*) throw () =0;
    /** teile die Spielerrolle mit */
    virtual void set_player_role (const char*) throw () =0;

    /** Einarbeitung der neuen Informationen in das Weltmodell initiieren */
    virtual void update () throw () =0;

    /** Weltmodell zuruecksetzen (falls es sich verloren hat) */
    virtual void reset () throw () =0;
    /** Weltmodell zuruecksetzen auf bestimmte Selbstlokalisierungsposition */
    virtual void reset (const Vec) throw () =0;
    /** Weltmodell zuruecksetzen auf bestimmte Selbstlokalisierungsposition und Orientierung */
    virtual void reset (const Vec, const Angle) throw () =0;

    /** Hinweis geben fuer SL auf Spielfeldsymmetrie, Arg1: ungefaehre Spielerposition */
    virtual void slMirrorHint (Vec) throw () =0;

    /** Log-Information in Log-Stream schreiben */
    virtual std::ostream& log_stream () throw () =0;
    /** Start einer neuen Iteration mitteilen, nur zu Protokollzwecken 
        Arg1: Zeitpunkt des Zyklusbeginns
        Arg2: Erwarteter Ausfuehrungszeitpunkt des Fahrtbefehls */
    virtual void init_cycle (Time, Time) throw () =0;
    /** diese Funktion wird vor der Kommunikation aufgerufen; zum Loggen des Messageboards */
    virtual void update_log () throw () {;}
    /** Das MessageBoard ausgehaendigt bekommen zur Kommunikation ueber WLAN mit dem Trainer und anderen Spielern */
    virtual MessageBoard& get_message_board () throw () =0;


    /** Liefert die aktuell erkannten Liniensegmente in Roboterkoordinaten, nur zu Testzwecken */
    virtual const std::vector<VisibleObjectList>& get_visible_objects () throw () =0;
    /** ein Fata-Morgana Objekt beim Weltmodell anmelden, nur fuer Test und Benchmarking */
    virtual void add_fata_morgana (FataMorgana* fm) throw () =0;
   };

}

#endif

