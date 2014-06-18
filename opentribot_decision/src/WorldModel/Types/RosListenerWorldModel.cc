
#include "RosListenerWorldModel.h"
#include "../WorldModelFactory.h"
#include "../../Structures/Journal.h"
#include "../WorldModel.h"
#include "ros/ros.h"
#include "opentribot_messages/WorldModel.h"













using namespace std;
using namespace Tribots;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public WorldModelBuilder {
    static Builder the_builder;
  public:
    Builder () {
      WorldModelFactory::get_world_model_factory ()->sign_up (string("RosListenerWorldModel"), this);
    }
    WorldModelType* get_world_model (const std::string&, const ConfigReader& reader, WorldModelType*) throw (TribotsException,bad_alloc) {
      return new RosListenerWorldModel (reader);
    }
  };
  Builder the_builder;
}



Tribots::RosListenerWorldModel::RosListenerWorldModel (const ConfigReader& vr) :WorldModelTypeBase (vr),  field_geometry (vr), null_stream ("/dev/null")  {
  cout << "Construction of RosListenerWorldModel"<<endl;
  /*game_state.in_game = false;
  game_state.refstate = stopRobot;
  */
  bool b;
  vector<double> darray (6);

    ball_loc.pos = Vec (darray[0], darray[1]);
    ball_loc.velocity = Vec (darray[2], darray[3]);
//    ball_loc.quality=0.8;
    ball_loc.pos_known=BallLocation::known;
    ball_loc.velocity_known=true;
    robot_loc.pos = Vec (darray[0], darray[1]);
    robot_loc.heading.set_deg (darray[2]);
    robot_loc.vtrans = Vec (darray[3], darray[4]);
    robot_loc.vrot = darray[5];


    ros::NodeHandle n;
    
    sub_worldmodel = n.subscribe("WorldModelMessages", 1, &Tribots::RosListenerWorldModel::receive_WorldModel,this);
    //sub =(ros::Subscriber) n.subscribe("GameStateMessages", 1, &Tribots::RosListenerWorldModel::receive_GameState,this);
    //sub =(ros::Subscriber) n.subscribe("TeamStateMessages", 1, &Tribots::RosListenerWorldModel::receive_TeamState,this);
    sub_refsig=(ros::Subscriber) n.subscribe("refsignals", 1,&Tribots::RosListenerWorldModel::receive_RefSig,this);
    sub_teamsig=(ros::Subscriber) n.subscribe("teamsignals", 1,&Tribots::RosListenerWorldModel::receive_TeamSig,this);





}

void Tribots::RosListenerWorldModel::reset (const Vec p) throw () {
  robot_loc.pos=p;
}

void Tribots::RosListenerWorldModel::reset (const Vec p, const Angle h) throw () {
  robot_loc.pos=p;
  robot_loc.heading=h;
}

RosListenerWorldModel::~RosListenerWorldModel () throw () {;}




void RosListenerWorldModel::init_cycle (Time t1 , Time t2 ) throw () {
  ball_relative.valid=false;
  
  WorldModelTypeBase::init_cycle (t1, t2);

}

RobotLocation RosListenerWorldModel::get_robot (Time t) const throw () {
   
  return robot_loc;
}

BallLocation RosListenerWorldModel::get_ball (Time t) const throw () {

	return ball_loc;
}

ObstacleLocation RosListenerWorldModel::get_obstacles (Time) const throw () {
  return obstacle_loc;
}


Time RosListenerWorldModel::get_timestamp_latest_update () const throw () {
  return Time();
}

void RosListenerWorldModel::receive_WorldModel(const opentribot_messages::WorldModel::ConstPtr& msg){

robot_loc.pos.x=msg->robot.pos.x;
robot_loc.pos.y=msg->robot.pos.y;
robot_loc.heading.set_rad(msg->robot.heading.theta);
robot_loc.kick=msg->robot.kick;
robot_loc.stuck.dir_of_stuck.x=msg->robot.stuck.dir_of_stuck.x;
robot_loc.stuck.dir_of_stuck.y=msg->robot.stuck.dir_of_stuck.y;
robot_loc.stuck.msec_since_stuck=msg->robot.stuck.msec_since_stuck;
robot_loc.stuck.pos_of_stuck.x=msg->robot.stuck.pos_of_stuck.x;
robot_loc.stuck.pos_of_stuck.y=msg->robot.stuck.pos_of_stuck.y;
robot_loc.stuck.robot_stuck=msg->robot.stuck.robot_stuck;
robot_loc.valid=msg->robot.valid;
robot_loc.vrot=msg->robot.vrot;
robot_loc.vtrans.x=msg->robot.vtrans.x;
robot_loc.vtrans.y=msg->robot.vtrans.y;
ball_loc.lastly_seen.set_sec(msg->ball.lastly_seen_sec);
ball_loc.lastly_seen.set_usec(msg->ball.lastly_seen_nsec);
ball_loc.pos.x=msg->ball.pos.x;

ball_loc.pos.y=msg->ball.pos.y;
ball_loc.pos.z=0;


ball_loc.pos_known=(Tribots::BallLocation::BallAttribute)msg->ball.pos_known;

ball_loc.velocity.x=msg->ball.velocity.x;
ball_loc.velocity.y=msg->ball.velocity.y;
ball_loc.velocity_known=msg->ball.velocity_known;



ObstacleDescriptor d;

for (int i=0;i< msg->obstacles.obstacles.size();i++){
  d.player=msg->obstacles.obstacles[i].player;
  d.pos.x=msg->obstacles.obstacles[i].posx;
  d.pos.y=msg->obstacles.obstacles[i].posy;
  d.velocity.x=msg->obstacles.obstacles[i].velx;
  d.velocity.y=msg->obstacles.obstacles[i].vely;
  d.width=msg->obstacles.obstacles[i].width;
  obstacle_loc.push_back(d);
 }



//the_world_model->set_robot_location(rloc);
//the_world_model->set_ball_location(bloc);






}

void RosListenerWorldModel::receive_RefSig(const opentribot_messages::RefSignal::ConstPtr& msg){
cout << "received RefBoxSignal "<<refbox_signal_names[msg->refsig]<<endl;
	this->update_refbox((Tribots::RefboxSignal)msg->refsig);
}

void RosListenerWorldModel::receive_TeamSig(const opentribot_messages::TeamSignal::ConstPtr& msg){
cout << "received Teamsignal "<<msg->teamsignal<<"  "<<teamsignal_names[msg->teamsignal]<<endl;
if (msg->teamsignal==TeamSIGstop)startstop(false);
else startstop(true);

gsman.set_team_signal(msg->teamsignal);

}

  
  
  
  


void RosListenerWorldModel::receive_GameState(const opentribot_messages::GameState::ConstPtr& msg){

cout <<"Setting Ros Gamestate"<<endl;

// put stuff into worldmodel

/*game_state.actual_cycle_time=msg->actual_cycle_time;
game_state.cycle_num=msg->cycle_num;
game_state.in_game=msg->in_game;
game_state.opponent_score=msg->opponent_score;
game_state.own_score=msg->own_score;
game_state.refstate=(Tribots::RefereeState)msg->refstate;
*/



//the_world_model->set_robot_location(rloc);
//the_world_model->set_ball_location(bloc);






}

void Tribots::RosListenerWorldModel::update () throw () {

/*  update_game_state ();  // game state aktualisieren
  
    unsigned int updated=update_localisation ();  // Lokalisierung, Ballfilter etc. aktualisieren
  
  Time tref = get_timestamp_latest_update ();
*/
WorldModelTypeBase::update();


  // Ball eintragen:
 Vec brel;
  brel=(ball_loc.pos.toVec()-robot_loc.pos)/robot_loc.heading;
  if (ball_loc.pos_known) {
    ball_relative.pos = brel;
    ball_relative.valid=true;
        
      
  }
 

 

}
