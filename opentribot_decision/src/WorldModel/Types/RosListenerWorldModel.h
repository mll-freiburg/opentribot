
#ifndef _Tribots_RosListenerWorldModel_h_
#define _Tribots_RosListenerWorldModel_h_

#include "WorldModelTypeBase.h"
#include <fstream>
#include "opentribot_messages/WorldModel.h"
#include <iostream>
#include "ros/ros.h"

#include "opentribot_messages/RefSignal.h"
#include "opentribot_messages/TeamSignal.h"
#include "opentribot_messages/GameState.h"

namespace Tribots {








  /** WorldModelDummy liest Ball- und Roboterposition aus Skriptfile und liefert stets diese Werte */
  class RosListenerWorldModel:public WorldModelTypeBase {
  private:
    RobotLocation robot_loc;  //wird uebertragen  
    BallLocation ball_loc;     //wird uebertragen
    ObstacleLocation obstacle_loc; //wird uebertragen
    FieldGeometry field_geometry; // wird durch config geladen
    //GameState game_state;  //wird uebertragen
    RobotProperties robot_properties;  //wird durch config geladen
    RobotData robot_data;
    std::ofstream null_stream;
    MessageBoard mboard;
    std::vector<VisibleObjectList> vol;
    DriveVector drive_vector;
    std::vector<TeammateLocation> teammates;
    std::string behavior;
    ros::Subscriber sub_worldmodel;
    ros::Subscriber sub_refsig;
    ros::Subscriber sub_teamsig;

  public:
    RosListenerWorldModel(const ConfigReader& vr);
    ~RosListenerWorldModel () throw ();
    const RobotLocation& get_robot_location (Time, bool) throw () { return robot_loc; }
    const BallLocation& get_ball_location (Time, bool=true) throw () { return ball_loc; }
    const ObstacleLocation& get_obstacle_location (Time, bool) throw () { return obstacle_loc; }
    const FieldGeometry& get_field_geometry () const throw () { return field_geometry; }
  
    //const GameState& get_game_state () const throw () { return game_state; }
    const RobotLocation& get_slfilter_robot_location (Time&) const throw () { return robot_loc; }
    const RobotProperties& get_robot_properties () const throw () { return robot_properties; }
    const RobotData& get_robot_data (Time&) const throw () { return robot_data; }
     
//    void set_robot_properties (const RobotProperties&) throw () {;}
//    void set_robot_data (const RobotData&, Time) throw() {;}
  //  void set_drive_vector (DriveVector dv, Time) throw () { drive_vector = dv; }
    unsigned int update_localisation () throw () {;}
    void reset () throw () {;}
    void reset (const Vec) throw ();
    void reset (const Vec, const Angle) throw ();
//    void update () throw ();
    std::ostream& log_stream () throw () { return null_stream; }
    void init_cycle (Time, Time) throw ();
    MessageBoard& get_message_board () throw () { return mboard; }
    const std::vector<VisibleObjectList>& get_visible_objects () throw () { return vol; }
    void add_fata_morgana (FataMorgana*) throw () {;}
    const std::vector<TeammateLocation>& get_teammate_location () const throw () { return teammates; }
 //   void set_teammates (std::vector<TeammateLocation>&) throw () {;}
 //   void set_robot_id (unsigned int) throw () {;}
 //   unsigned int get_robot_id () throw () { return 0; }
    
 //   void set_score (unsigned int, unsigned int, unsigned int) throw () {;}
  //  void slMirrorHint (Vec v) throw () {;}
    void set_active_behavior (const char* n) throw () { behavior=n; }
    const char* get_active_behavior () const throw () { return behavior.c_str(); }
 //   void set_player_type (const char*) throw () {;}
//    void set_player_role (const char*) throw () {;}
    RobotLocation get_robot (Time) const throw ();
    BallLocation get_ball (Time) const throw ();
    ObstacleLocation get_obstacles (Time) const throw ();
    Time get_timestamp_latest_update () const throw ();
    void update () throw ();
    void receive_WorldModel(const opentribot_messages::WorldModel::ConstPtr& msg);
    void receive_GameState(const opentribot_messages::GameState::ConstPtr& msg);

    void receive_RefSig(const opentribot_messages::RefSignal::ConstPtr& msg);
    void receive_TeamSig(const opentribot_messages::TeamSignal::ConstPtr& msg);
      /*  inline void set_robot_location(RobotLocation&rl){robot_loc=rl;}
    inline void set_ball_location(BallLocation&bl){ball_loc=bl;}
*/

  };

}

#endif

