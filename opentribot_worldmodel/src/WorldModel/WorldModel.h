
#ifndef _Tribots_WorldModel_h_
#define _Tribots_WorldModel_h_

#include "WorldModelType.h"
#include "../Structures/TribotsException.h"
#include "../Structures/MessageBoard.h"
#include "../Fundamental/Vec3D.h"
#include <iostream>
#include <vector>

// #define DEBUG_NAN

#define MWM Tribots::WorldModel::get_main_world_model()
#define LOUT Tribots::WorldModel::get_main_world_model().log_stream()


namespace Tribots {

  /** Schnittsetelle des Weltmodells nach aussen
      Aufgaben: Verwaltung allgemeiner Information, Fortschreiben des Weltzustandes */
  class WorldModel {
  private:
    WorldModelType* the_world_model;
    char* world_model_descriptor;
    const ConfigReader& configuration_list;

    /** eigentliche Implementierung des Weltmodellwechsels 
        Arg1: Bezeichner, Arg2=Parametersatz, Arg3=bisheriges Weltmodell oder NULL */
    void really_change_world_model_type (const char*, const ConfigReader&) throw (TribotsException,std::bad_alloc);

    /** Zeiger auf das zentrale Weltmodell */
    static WorldModel* main_world_model;
  public:
    /** Moeglichkeit, sich eine Referenz auf das zentrale Weltmodell zu besorgen; arbeitet erst korrekt, nachdem ein erstes Weltmodell von ControlLoop erzeugt wurde */
    static WorldModel& get_main_world_model () throw ();
    static bool is_main_world_model_available () throw ();

    /** Konstruktor liest allerlei Parameter aus der Konfigurationsdatei */
    WorldModel (const ConfigReader&) throw (TribotsException,std::bad_alloc);
    ~WorldModel () throw ();


    // Beeinflussung des Weltbildtyps:
    /** Wechsel des Weltmodelltyps
        Weltmodell-Parameter werden aus dem ConfigReader gelesen, der mit dem Konstruktor uebergeben wurde
        Arg1: Bezeichner fuer Weltmodell */
    bool change_world_model_type (const char*) throw ();
    /** Wechsel des Weltmodelltyps
        Arg1: Bezeichner fuer Weltmodell
        Arg2: Parameterliste fuer neues Weltmodell */
    bool change_world_model_type (const char*, const ConfigReader&) throw ();

    /** liefere eine Beschreibung des aktuellen Weltmodelltyps */
    const char* get_world_model_type () const throw ();



    // Informationsgewinnung:
    /** liefere Spielfeldgeometrie */
    inline const FieldGeometry& get_field_geometry () const throw () { return the_world_model->get_field_geometry(); }
    /** liefere Orientierung des Spielfeldes (eigene Haelfte), +1, wenn Spiel von gelbem Tor auf blaues Tor, sonst -1 */
    inline int get_own_half () const throw () { return the_world_model->get_own_half(); }
    /** liefere letzten Spielzustand */
    inline const GameState& get_game_state () const throw () { return the_world_model->get_game_state(); }
    /** liefere Roboterposition, Geschwindigkeit und Orientierung zu einem Zeitpunkt (in Weltkoordinaten); 
        Arg1: Zeitpunkt, Arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    inline const RobotLocation& get_robot_location (Time t, bool ia =true) throw () { return the_world_model->get_robot_location(t, ia); }
    /** die Positionen der Mitspieler liefern (sofern bekannt) */
    inline const std::vector<TeammateLocation>& get_teammate_location () throw () { return the_world_model->get_teammate_location (); }
    /** Die Roboter-ID anfragen (Laetzchen-nummer) */
    inline unsigned int get_robot_id () throw () { return the_world_model->get_robot_id (); }
    /** liefere Ballposition und Geschwindigkeit zu einem Zeitpunkt (in Weltkoordinaten):
        Arg1: Zeitpunkt, Arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    inline const BallLocation& get_ball_location (Time t, bool ia =true) throw () { 
#ifndef DEBUG_NAN 
      return the_world_model->get_ball_location(t, ia); 
#else
      const BallLocation& ball = the_world_model->get_ball_location(t, ia);
      if (ball.vtrans.x != ball.vtrans.x || ball.vrot != ball.vrot ||
          ball.pos.x != ball.pos.x) {
        LOUT << "NAN_ERROR: In world model: get_ball_location." << endl;
      }
      return ball;
#endif
    }
    /** die relative, ungefilterte Ballposition anfragen */
    inline const BallRelative& get_ball_relative () throw () { return the_world_model->get_ball_relative(); }
    /** liefere Position der Hindernisse;
        Arg1: Zeitpunkt, Arg2: sollen Objektinteraktionen beruecksichtigt werden? */
    inline const ObstacleLocation& get_obstacle_location (Time t, bool ia = true) throw () { return the_world_model->get_obstacle_location(t, ia); }
    /** liefere Robotereigenschaften */
    inline const RobotProperties& get_robot_properties () const throw () { return the_world_model->get_robot_properties(); }
    /** liefere zusaetzliche Daten des Roboters
        Arg1: Referenz auf Zeitobjekt in das die Zeit geschrieben wird von dem die Daten stammen */
    inline const RobotData& get_robot_data (Time& t) const throw () { return the_world_model->get_robot_data (t);};
    /** liefere eine aus der Selbstlokalisierung geschaetzte Geschwindigkeit 
        Arg1: Referenz auf Zeitobjekt in das die Zeit geschrieben wird von dem die Daten stammen */
    inline const RobotLocation& get_slfilter_robot_location (Time& t) const throw () {
#ifndef DEBUG_NAN 
      return the_world_model->get_slfilter_robot_location (t);
#else
      const RoboLocation& robot = the_world_model->get_slfilter_robot_location (t);
      if (robot.vtrans.x != robot.vtrans.x || robot.vrot != robot.vrot ||
          robot.pos.x != robot.pos.x) {
        LOUT << "NAN_ERROR: In world model: get_robot_location." << endl;
      }
      return robot;
#endif
    }
    /** liefere den zuletzt gesetzten Fahrtvektor */
    inline const DriveVector& get_recent_drive_vector () const throw () { return the_world_model->get_recent_drive_vector (); }
    /** liefert das zuletzt aktive Behavior, wenn bekannt */
    inline const char* get_active_behavior () const throw () { return the_world_model->get_active_behavior (); }


    // Informationen einfliessen lassen:
    /** Spielrichtung setzen; Arg1: +1, wenn Spiel von gelbem Tor auf blaues Tor, sonst -1 */
    inline void set_own_half (int h) throw () { the_world_model->set_own_half(h); }
    /** Signal der Refereebox einarbeiten */
    inline void update_refbox (RefboxSignal sig) throw () { the_world_model->update_refbox(sig); }
    /** Spielstand (arg1 eigene, arg2 gegnerische Tore) und gelbe Karten (arg3) setzen */
    inline void set_score (unsigned int own_score, unsigned int opponent_score, unsigned int yellow_cards) throw () { the_world_model->set_score (own_score, opponent_score, yellow_cards); }
    /** Roboter starten (true)/stoppen(false), veraendern des "in_game"-Flags */
    inline void startstop (bool b) throw () { the_world_model->startstop (b); }
    /** neuen Fahr-/Kickvektor mitteilen
        Arg1: Fahrvektor
        Arg2: Zeitpunkt, zu dem der Fahrvektor gesetzt wurde */
    inline void set_drive_vector (DriveVector dv, Time t) throw () { the_world_model->set_drive_vector(dv,t); }
    /** gemessene Bewegung mitteilen
        Arg1: gemessener Fahrvektor
        Arg2: Zeitpunkt der Messung */
    inline void set_odometry (DriveVector dv, Time t) throw () { the_world_model->set_odometry(dv,t); }
    /** gemessene Rotation mitteilen
        Arg1: gemessene Rotation
        Arg2: Zeitpunkt der Messung */
    inline void set_gyro_data (GyroData gd, Time t) throw () { the_world_model->set_gyro_data (gd,t); }
    /** neue Information des visuellen Sensors integrieren
        Arg1: Objektliste 
        Arg2: Bildquellen-ID */
    inline void set_visual_information (const VisibleObjectList& vol, unsigned int camera=0) throw () { for (unsigned int i=0; i<vol.objectlist.size(); i++) the_world_model->set_visual_information(vol.objectlist[i], vol.timestamp, camera); }
    /** neue Information des visuellen Sensors integrieren
        Arg1: Einzelnes Objekt
        Arg2: Zeitstempel fuer dieses Objekt (Zeitpunkt der Aufnahme)
        Arg3: Bildquellen-ID */
    inline void set_visual_information (const VisibleObject& vo, Time t, unsigned int camera=0) throw () { the_world_model->set_visual_information(vo,t,camera); }
    /** trage Robotereigenschaften ein */
    inline void set_robot_properties (const RobotProperties& rpr) throw () { the_world_model->set_robot_properties (rpr); }
    /** neue Roboterdaten setzen, diese werden im Moment nicht weitergerechnet 
        Arg1: Roboterdaten
        Arg2: Zeitpunkt von dem die Daten stammen */
    inline void set_robot_data (const RobotData& rd, Time t) throw() {the_world_model->set_robot_data ( rd, t);};
    /** neu uebermittelte Positionen der Mitspieler einbinden */
    inline void set_teammates (std::vector<TeammateLocation>& tlo) throw () { the_world_model->set_teammates (tlo); }
    /** Roboter-ID setzen */
    inline void set_robot_id (unsigned int id) throw () { the_world_model->set_robot_id (id); }
    /** teile das zuletzt aktive Behavior mit, wenn anwendbar */
    inline void set_active_behavior (const char* n) throw () { the_world_model->set_active_behavior (n); }
    /** teile den Spielertyp mit */
    inline void set_player_type (const char* n) throw () { the_world_model->set_player_type (n); }
    /** teile die Spielerrolle mit */
    inline void set_player_role (const char* n) throw () { the_world_model->set_player_role (n); }

    /** Einarbeitung der neuen Informationen in das Weltmodell initiieren */
    inline void update () throw () { the_world_model->update(); }
    /** Weltmodell zuruecksetzen (falls es sich verloren hat) */
    inline void reset () throw () { the_world_model->reset(); }
    /** Weltmodell zuruecksetzen auf bestimmte Selbstlokalisierungsposition */
    inline void reset (const Vec p) throw () { the_world_model->reset(p); }
    /** Weltmodell zuruecksetzen auf bestimmte Selbstlokalisierungsposition und Orientierung */
    inline void reset (const Vec p , const Angle a) throw () { the_world_model->reset(p, a); }
    /** Hinweis geben fuer SL auf Spielfeldsymmetrie, Arg1: ungefaehre Spielerposition */
    inline void slMirrorHint (Vec v) throw () { the_world_model->slMirrorHint (v); }


    /** Log-Information in Log-Stream schreiben, siehe auch Define LOUT */
    inline std::ostream& log_stream () throw () { return the_world_model->log_stream (); }
    /** Start einer neuen Iteration mitteilen, nur zu Protokollzwecken 
        Arg1: Zeitpunkt des Zyklusbeginns
        Arg2: erwartete Ausfuehrungszeit des Fahrbefehls */
    inline void init_cycle (Time t1, Time t2) throw () { the_world_model->init_cycle(t1, t2); }
    /** diese Funktion wird vor der Kommunikation aufgerufen; zum Loggen des Messageboards */
    inline void update_log () throw () { the_world_model->update_log(); }
    /** Das MessageBoard ausgehaendigt bekommen zur Kommunikation ueber WLAN mit dem Trainer und anderen Spielern */
    inline MessageBoard& get_message_board () throw () { return the_world_model->get_message_board(); }

    /** ein Fata-Morgana Objekt beim Weltmodell anmelden, nur fuer Test und Benchmarking */
    inline void add_fata_morgana (FataMorgana* fm) throw () { the_world_model->add_fata_morgana (fm); }
  protected:
    /** die Liste erkannter Objekte im Bild in Roboterkoordinaten liefern; diese Methode dient nur dem Zwecke, Linien im Teamcontrol 
     anzeigen zu koennen, um fehlerhafte Einstellungen festzustellen; daher protected mit friend; die Methode ist evtl. nicht
    bei allen Weltmodelltypen sinnvoll implementiert */
    inline const std::vector<VisibleObjectList>& get_visible_objects () throw () { return the_world_model->get_visible_objects(); }

    friend class AddComUserInterface;
  };

}


#endif

