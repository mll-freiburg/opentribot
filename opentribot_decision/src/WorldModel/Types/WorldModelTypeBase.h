
#ifndef _Tribots_WorldModelTypeBase_h_
#define _Tribots_WorldModelTypeBase_h_

#include "../WorldModelType.h"
#include "../../Fundamental/ConfigReader.h"
#include "../Orga/GameStateManager.h"
#include "../Orga/VisualContainer.h"
#include "../Orga/OdometryContainer.h"
#include "../Prediction/VelocityPredictor.h"
#include "../SL/SLVelocitySensor.h"
#include "../SL/SLStuckSensor.h"
#include <fstream>


namespace Tribots {

  class LocationShortTermMemory;
  class FataMorgana;

  /** Implementierung einiger Funktionen des WorldModelTypes wie Verwaltung
      der Feld- und Roboterdaten, Gamestates, SLVelocitySensor und SLStuckSensor */
  class WorldModelTypeBase : public WorldModelType {
  protected:
    FieldGeometry field_geometry;                ///< die Feldgeometrie
    GameStateManager gsman;                      ///< die Verwaltungsstruktur fuer GameStates
    MessageBoard mboard;                         ///< das Messageboard
    int own_half;                                ///< eigene Haelfte: +1 , wenn Spiel von gelbem Tor auf blaues Tor, sonst -1
    RobotProperties robot_properties;            ///< Robotereigenschaften
    RobotData robot_data;                        ///< Zusätzliche Daten des Roboters (Temperatur der Motoren, Ströme, usw.)
    Time robot_data_time;
    DriveVector recent_drive_vector;             ///< zuletzt gesetzter DriveVector
    Time recent_drive_vector_time;
    Time recent_odometry_time;

    std::vector<VisibleObjectList> all_visible_objects [2];   ///< 2 Listen aller gesehenen Objekte, werden abwechselnd gesetzt
    unsigned int even_cycle;                     ///< abwechselnd 0 und 1 um den Zugriff auf all_visible_objects zu steuern
    std::vector<VisualContainer> visbox;         ///< aktuelle Objektlisten (fuer update_localisation ())
    OdometryContainer odobox;                    ///< Odometrie und Fahrtvektoren
    BallRelative ball_relative;                  ///< Relativer Ball, ungefiltert

    LocationShortTermMemory* locations;          ///< Pufferspeicher fuer Robot-, Ball- und ObstacleLocation
    VelocityPredictor* velocity_predictor;       ///< Vorhersage fuer Robotergeschwindigkeiten

    std::ofstream null_stream;                   ///< Stream auf /dev/null

    bool tournament_mode;                        ///< der Wert aus dem Config-File; falls true, werden keine FataMorgana-Objekte akzeptiert
    std::vector<FataMorgana*> fata_morgana;      ///< Liste aller FataMorgana-Objekte

    SLVelocitySensor velocity_filter;            ///< Geschwindigkeitsschaetzung aus Selbstlokalisierung
    SLStuckSensor stuck_sensor;                  ///< Erkennung von Blockadesituationen
    std::vector<TeammateLocation> teammates;     ///< Position der Mitspieler, falls bekannt
    unsigned int robot_id;                       ///< Laetzchen-Nummer
    std::string behavior;                        ///< aktives Behavior, falls bekannt

    /** der Teil des Updates, der sich um den Gamestate kuemmert */
    virtual void update_game_state () throw ();
    /** der Teil des Updates, der sich um Roboter-, Ball- und Hindernisposition kuemmert;
        Rueckgabewert kodiert binaer, ob tatsaechlich ein Update erfolgen konnte:
            ret&1==1 wenn SL aktualisiert werden konnte,
            ret&2==2 wenn Ballposition aktualisiert werden konnte,
            ret&4==4 wenn Hindernisse aktualisiert werden konnten */
    virtual unsigned int update_localisation () throw () =0;

  public:
    WorldModelTypeBase (const ConfigReader&) throw (InvalidConfigurationException, std::bad_alloc);
    ~WorldModelTypeBase () throw ();

    const FieldGeometry& get_field_geometry () const throw ();
    int get_own_half () const throw ();
    const GameState& get_game_state () const throw ();
    const RobotProperties& get_robot_properties () const throw ();
    const RobotData& get_robot_data (Time&) const throw ();
    const RobotLocation& get_slfilter_robot_location (Time&) const throw ();
    const DriveVector& get_recent_drive_vector () const throw ();
    const std::vector<TeammateLocation>& get_teammate_location () const throw ();
    const BallRelative& get_ball_relative () throw ();
    const char* get_active_behavior () const throw ();

    // Im WorldModelTypeBase werden die Objektinteraktionen untersucht sowie eine 
    // Zwischenspeicherung fuer haeufig abgefragte Zeitpunkte realisiert
    // Dazu werden die Methoden get_robot_location, get_ball_location und 
    // get_obstacle_location realisiert. Um an die nicht-interagierten Positionen zu kommen, 
    // wird auf die Methoden get_robot, get_ball und get_obstacles zugegriffen
    virtual RobotLocation get_robot (Time) const throw () =0;
    virtual BallLocation get_ball (Time) const throw () =0;
    virtual ObstacleLocation get_obstacles (Time) const throw () =0;

    const RobotLocation& get_robot_location (Time, bool) throw ();
    const BallLocation& get_ball_location (Time, bool) throw ();
    const ObstacleLocation& get_obstacle_location (Time, bool) throw ();
    unsigned int get_robot_id () throw ();
    virtual Time get_timestamp_latest_update () const throw () =0;   ///< liefere Zeitpunkt, fuer den die Selbstlokalisation zum letzten Mal aktualisiert wurde

    void set_own_half (int) throw ();
    void update_refbox (RefboxSignal) throw ();
    void set_score (unsigned int, unsigned int, unsigned int) throw ();
    void startstop (bool) throw ();
    void set_robot_properties (const RobotProperties&) throw ();
    void set_robot_data (const RobotData&, Time) throw();
    void set_teammates (std::vector<TeammateLocation>&) throw ();
    void set_robot_id (unsigned int) throw ();
    void set_active_behavior (const char*) throw ();
    void set_player_type (const char*) throw () {;}
    void set_player_role (const char*) throw () {;}

    void reset () throw () =0;
    void reset (const Vec) throw () =0;
    void reset (const Vec, const Angle) throw ();
    void slMirrorHint (Vec) throw () {;}  ///< default-Implementierung, tut nichts
    void update () throw ();

    std::ostream& log_stream () throw ();
    void init_cycle (Time, Time) throw ();
    MessageBoard& get_message_board () throw ();

    void set_drive_vector (DriveVector, Time) throw ();
    void set_odometry (DriveVector, Time) throw ();
    void set_gyro_data (GyroData, Time) throw ();
    void set_visual_information (const VisibleObject&, Time, unsigned int) throw ();
    const std::vector<VisibleObjectList>& get_visible_objects () throw ();

    /** ein Fata-Morgana Objekt beim Weltmodell anmelden */
    void add_fata_morgana (FataMorgana*) throw ();
    /** einen kuenstlichen Ball im Weltmodell eintragen an robzentrischer Position */
    void add_ball_relative (Vec) throw ();
    /** einen kuenstlichen Ball im Weltmodell eintragen an globaler Position */
    void add_ball_absolute (Vec) throw ();
    /** ein kuenstliches Hindernis im Weltmodell eintragen an robzentrischer Position mit angegebener Breite */
    void add_obstacle_relative (Vec, double) throw ();
    /** ein kuenstliches Hindernis im Weltmodell eintragen an globaler Position mit angegebener Breite */
    void add_obstacle_absolute (Vec, double) throw ();



  private:
    // Debug-Moeglichkeit fuer die Geschwindigkeitsprognose:
    std::ofstream* prediction_error_out;   // Ausgabestream
    int prediction_testdelay;  // Delay (in ms), fuer den Ausgaben erzeugt werden sollen
    struct TRL {
      Time timestamp;
      RobotLocation rloc;
    };
    RingBuffer<TRL> old_predictions;  // Ringpuffer zum Zwischenspeichern von Vorhersagen
  };

}

#endif

