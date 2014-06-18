
#ifndef _Tribots_SharedWorldModel_h_
#define _Tribots_SharedWorldModel_h_

#include "WorldModelTypeBase.h"
#include <fstream>

namespace Tribots {

  /** WorldModelDummy liest Ball- und Roboterposition aus Skriptfile und liefert stets diese Werte */
  class SharedWorldModel:public WorldModelTypeBase {
  private:
    RobotLocation robot_loc;
    BallLocation ball_loc;
    ObstacleLocation obstacle_loc;
    FieldGeometry field_geometry;
    GameState game_state;
    RobotProperties robot_properties;
    RobotData robot_data;
    std::ofstream null_stream;
    MessageBoard mboard;
    std::vector<VisibleObjectList> vol;
    DriveVector drive_vector;
    std::vector<TeammateLocation> teammates;
    BallRelative ball_relative;
    std::string behavior;

  public:
    SharedWorldModel(const ConfigReader& vr);
    ~SharedWorldModel () throw ();
    const RobotLocation& get_robot_location (Time, bool) throw () { return robot_loc; }
    const BallLocation& get_ball_location (Time, bool) throw () { return ball_loc; }
    const ObstacleLocation& get_obstacle_location (Time, bool) throw () { return obstacle_loc; }
    const FieldGeometry& get_field_geometry () const throw () { return field_geometry; }
    int get_own_half () const throw () { return 1; }
    const GameState& get_game_state () const throw () { return game_state; }
    const RobotLocation& get_slfilter_robot_location (Time&) const throw () { return robot_loc; }
    const RobotProperties& get_robot_properties () const throw () { return robot_properties; }
    const RobotData& get_robot_data (Time&) const throw () { return robot_data; }
    void set_own_half (int) throw () {;}
    void update_refbox (RefboxSignal) throw ();
    void startstop (bool) throw ();
    void set_robot_properties (const RobotProperties&) throw () {;}
    void set_robot_data (const RobotData&, Time) throw() {;}
    void set_drive_vector (DriveVector dv, Time) throw () { drive_vector = dv; }
    void set_odometry (DriveVector, Time) throw () {;}
    void set_gyro_data (GyroData, Time) throw () {;}
    void set_visual_information (const VisibleObject& v, Time, unsigned int) throw ();
    unsigned int update_localisation () throw () {;}
    void reset () throw () {;}
    void reset (const Vec) throw ();
    void reset (const Vec, const Angle) throw ();
    void update () throw () {;}
    std::ostream& log_stream () throw () { return null_stream; }
    void init_cycle (Time, Time) throw ();
    MessageBoard& get_message_board () throw () { return mboard; }
    const std::vector<VisibleObjectList>& get_visible_objects () throw () { return vol; }
    void add_fata_morgana (FataMorgana*) throw () {;}
    const DriveVector& get_recent_drive_vector () const throw () { return drive_vector; }
    const std::vector<TeammateLocation>& get_teammate_location () const throw () { return teammates; }
    void set_teammates (std::vector<TeammateLocation>&) throw () {;}
    void set_robot_id (unsigned int) throw () {;}
    unsigned int get_robot_id () throw () { return 0; }
    const BallRelative& get_ball_relative () throw () { return ball_relative; }
    void set_score (unsigned int, unsigned int, unsigned int) throw () {;}
    void slMirrorHint (Vec v) throw () {;}
    void set_active_behavior (const char* n) throw () { behavior=n; }
    const char* get_active_behavior () const throw () { return behavior.c_str(); }
    void set_player_type (const char*) throw () {;}
    void set_player_role (const char*) throw () {;}
  RobotLocation get_robot (Time) const throw ();
    BallLocation get_ball (Time) const throw ();
    ObstacleLocation get_obstacles (Time) const throw ();
    Time get_timestamp_latest_update () const throw ();

  };

}

#endif

