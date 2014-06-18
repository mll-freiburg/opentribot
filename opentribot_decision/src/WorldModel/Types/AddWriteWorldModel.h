
#ifndef _Tribots_AddWriteWorldModel_h_
#define _Tribots_AddWriteWorldModel_h_


#include "../WorldModelType.h"
#include "../../Structures/VisibleObjectReadWriter.h"
#include "../../Structures/DriveVectorReadWriter.h"
#include "../../Structures/GameStateReadWriter.h"
#include "../../Structures/RobotLocationReadWriter.h"
#include "../../Structures/BallLocationReadWriter.h"
#include "../../Structures/ObstacleLocationReadWriter.h"
#include "../../Structures/GyroDataReadWriter.h"
#include "../../Structures/MessageBoardReadWriter.h"
#include <iostream>


namespace Tribots {

  /** Klasse AddWriteWorldModel stellt Funktionalitaet zum Mitprotokollieren
      der eingegangen Informationen und geschaetzten Positionen bereit;
      ansonsten bindet es ein zweites, richtiges Weltmodell mit ein */
  class AddWriteWorldModel : public WorldModelType {
  private:
    WorldModelType* the_world_model;    // das eigentliche Weltmodell
    std::ostream* odometry_out;         // diverse Ausgabestreams
    std::ostream* drive_vector_out;
    std::ostream* visual_info_out;
    std::ostream* robot_pos_out;
    std::ostream* ball_pos_out;
    std::ostream* obs_pos_out;
    std::ostream* log_out;
    std::ostream* gs_out;
    std::ostream* rawrobotdata_out;
    std::ostream* gyrodata_out;
    std::ostream* messageboard_out;

    bool first_visual;                  // die erste Bildinformation eines Zykluses?
    Time first_visual_timestamp;        // erster Zeitstempel fuer Bildinformation des Zykluses
    Time image_timestamp;               // Alter des Bildes

    VisibleObjectWriter* visual_writer; // diverse Ausgabeobjekte
    DriveVectorWriter* drv_writer;
    DriveVectorWriter* odo_writer;
    GameStateWriter* gs_writer;
    RobotLocationWriter* robot_writer;
    BallLocationWriter* ball_writer;
    ObstacleLocationWriter* obstacle_writer;
    GyroDataWriter* gyrodata_writer;
    MessageBoardWriter* messageboard_writer;

    double average_vision_delay;
    bool obtained_visual_information;

    std::string playertype;
    std::string playerrole;
    std::string behavior;
    GameState latest_game_state;  // game states werden erst im darauffolgenden Zyklus geschrieben, da behavior erst nach WorldModel::update() bekannt wird
    std::string latest_playerrole;
    std::string latest_playertype;
    Time latest_timestamp;
  public:
    /** Konstruktor; arg1 enthaelt die Dateinamen, arg2 das eigentliche Weltmodell */
    AddWriteWorldModel (const ConfigReader&, WorldModelType*) throw ();
    ~AddWriteWorldModel () throw ();

    void set_drive_vector (DriveVector, Time) throw ();
    void set_odometry (DriveVector, Time) throw ();
    void set_gyro_data (GyroData, Time) throw ();
    void set_visual_information (const VisibleObject&, Time, unsigned int) throw ();
    void update () throw ();
    /** Log-Stream */
    std::ostream& log_stream () throw ();   
    /** Start einer Iteration mitteilen, nur zu Protokollzwecken
        Arg1: Zeitpunkt des Zyklusbeginns
        Arg2: Erwarteter Ausfuehrungszeitpunkt des Fahrtbefehls */
    void init_cycle (Time, Time) throw ();
    void update_log () throw ();

    // sontige Methoden, die lediglich durchreichen:
    const RobotLocation& get_robot_location (Time, bool) throw ();
    const BallLocation& get_ball_location (Time, bool) throw ();
    const ObstacleLocation& get_obstacle_location (Time, bool) throw ();
    void reset () throw ();
    void reset (const Vec) throw ();
    void reset (const Vec, const Angle) throw ();
    const FieldGeometry& get_field_geometry () const throw () { return the_world_model->get_field_geometry (); }
    int get_own_half () const throw () { return the_world_model->get_own_half (); }
    const GameState& get_game_state () const throw () { return the_world_model->get_game_state (); }
    const RobotProperties& get_robot_properties () const throw () { return the_world_model->get_robot_properties (); }
    const RobotData& get_robot_data (Time& t) const throw () { return the_world_model->get_robot_data (t); }
    void set_own_half (int d) throw () { the_world_model->set_own_half (d); }
    void update_refbox (RefboxSignal sig) throw () { the_world_model->update_refbox (sig); }
    void startstop (bool b) throw () { the_world_model->startstop (b); }
    void set_robot_properties (const RobotProperties& rp) throw () { the_world_model->set_robot_properties (rp); }
    void set_robot_data (const RobotData& rd, Time t) throw();
    const RobotLocation& get_slfilter_robot_location (Time&) const throw ();
    MessageBoard& get_message_board () throw ();
    const std::vector<VisibleObjectList>& get_visible_objects () throw () { return the_world_model->get_visible_objects(); }
    void add_fata_morgana (FataMorgana* fm) throw () { the_world_model->add_fata_morgana (fm); }
    const DriveVector& get_recent_drive_vector () const throw () { return the_world_model->get_recent_drive_vector (); }
    const std::vector<TeammateLocation>& get_teammate_location () const throw () { return the_world_model->get_teammate_location (); }
    void set_teammates (std::vector<TeammateLocation>& tlo) throw () { return the_world_model->set_teammates (tlo); }
    void set_robot_id (unsigned int id) throw () { the_world_model->set_robot_id (id); }
    unsigned int get_robot_id () throw () { return the_world_model->get_robot_id (); }
    const BallRelative& get_ball_relative () throw () { return the_world_model->get_ball_relative (); }
    void set_score (unsigned int own, unsigned int opp, unsigned int yell) throw () { return the_world_model->set_score (own, opp, yell); }
    void slMirrorHint (Vec v) throw () { return the_world_model->slMirrorHint (v); }
    void set_active_behavior (const char*) throw ();
    const char* get_active_behavior () const throw ();
    void set_player_type (const char*) throw ();
    void set_player_role (const char*) throw ();
  };

}

#endif
