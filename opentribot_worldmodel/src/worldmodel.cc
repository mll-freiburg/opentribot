
#include "Structures/Journal.h"
#include "Fundamental/random.h"
#include <string>
#include <fstream>
#include <sys/types.h>
#include <unistd.h>
#include "WorldModel/WorldModel.h"
#include "ros/ros.h"
#include "opentribot_messages/VisibleObjectList.h"
#include "opentribot_messages/WorldModel.h"
#include "opentribot_messages/SimWorldModel.h"
#include "opentribot_messages/TribotDrive.h"
#include "UserInterface/UserInterface.h"


using namespace std;
using namespace Tribots;

//#define DEBUGCMDLINEARG 1
WorldModel * the_world_model;
UserInterface * the_user_interface;
ros::Publisher * drivecommands_sender;





void WorldModelCycle(const opentribot_messages::VisibleObjectList::ConstPtr& msg)
{


Tribots::VisibleObjectList list;
Tribots::VisibleObject elem;


for(int i=0;i<msg->objects.size();i++){
elem.pos.x=msg->objects[i].posx;
elem.pos.y=msg->objects[i].posy;
elem.z=msg->objects[i].posz;
elem.width=msg->objects[i].width;
elem.object_type=(Tribots::VisibleObject::ObjectType)msg->objects[i].type;
list.objectlist.push_back(elem);
} 


the_world_model->set_visual_information(list,0);

//cout <<" received "<<msg->objects.size()<<"elements... "<<endl;

the_world_model->update();

//cout << "Updated worldmodel"<<endl;

Time t;
// now we need to send the WorldModel data
opentribot_messages::WorldModel wmsg;
BallLocation bl=the_world_model->get_ball_location(t);
//cout << "Ball pos sent: "<<bl.pos<<endl;
wmsg.ball.pos.x=bl.pos.x;
wmsg.ball.pos.y=bl.pos.y;
wmsg.ball.pos.z=bl.pos.z;
wmsg.ball.pos_known=bl.pos_known;
wmsg.ball.velocity.x=bl.velocity.x;
wmsg.ball.velocity.y=bl.velocity.y;
wmsg.ball.velocity.z=bl.velocity.z;
wmsg.ball.velocity_known=bl.velocity_known;
wmsg.ball.lastly_seen_nsec=bl.lastly_seen.get_usec();
wmsg.ball.lastly_seen_sec=bl.lastly_seen.get_sec();
RobotLocation rl=the_world_model->get_robot_location(t);
wmsg.robot.pos.x=rl.pos.x;
wmsg.robot.pos.y=rl.pos.y;
wmsg.robot.kick=rl.kick;
wmsg.robot.heading.theta=rl.heading.get_rad();
wmsg.robot.stuck.dir_of_stuck.x=rl.stuck.dir_of_stuck.x;
wmsg.robot.stuck.dir_of_stuck.y=rl.stuck.dir_of_stuck.y;
wmsg.robot.stuck.msec_since_stuck=rl.stuck.msec_since_stuck;
wmsg.robot.stuck.pos_of_stuck.x =rl.stuck.pos_of_stuck.x;
wmsg.robot.stuck.pos_of_stuck.y =rl.stuck.pos_of_stuck.y;
wmsg.robot.stuck.robot_stuck=rl.stuck.robot_stuck;


wmsg.robot.valid=rl.valid;
wmsg.robot.vrot=rl.vrot;
wmsg.robot.vtrans.x=rl.vtrans.x;
wmsg.robot.vtrans.y=rl.vtrans.y;
ObstacleLocation ol=the_world_model->get_obstacle_location(t);
opentribot_messages::ObstacleDescriptor od;
for (int i=0;i<ol.size();i++){
  od.player=ol[i].player;
  od.posx=ol[i].pos.x;
  od.posy=ol[i].pos.y;
  od.velx=ol[i].velocity.x;
  od.vely=ol[i].velocity.y;
  od.width=ol[i].width;
  wmsg.obstacles.obstacles.push_back(od);
}




    drivecommands_sender->publish(wmsg);





the_user_interface->process_messages();










}





void setOdometry(const opentribot_messages::TribotDrive::ConstPtr& msg)
{
DriveVector dv;

dv.vtrans.x=msg->vtransx;
dv.vtrans.y=msg->vtransy;
dv.vrot=msg->vrot;
dv.kick=msg->kick;
dv.klength=msg->kicklength;


the_world_model->set_drive_vector(dv, Tribots::Time());


}


int main (int argc,  char** argv) {

   	Time timestamp;


 

  // recover previous Worldmodel position
  bool auto_restart=true;
  /** rotate_log: decides if time information is appended to logfile
   * names so logfiles are not lost on multiple calls of the program
   * (Communicated using configreader, read in AddWriteWorldModel)
   **/
  bool rotate_log = true;
  bool set_stream_interface = false;
  //std::string configfile = ("configs/robotcontrol.cfg");
  std::string configfile = ("config_files/robotcontrol.cfg");
  
  // Command line argument parsing:
  Tribots::ConfigReader cfg_cl (0);
  cfg_cl.append_from_command_line (argc, argv);

  string s;
  bool b;
  
  if (cfg_cl.get("stream_interface",b)) set_stream_interface=true;
  if (cfg_cl.get("ConfigReader::unknown_argument_1", s))
    configfile=s;

#ifdef DEBUGCMDLINEARG
  std::cout << "auto_restart: " << auto_restart << "\n"
          << "rotate_log: " << rotate_log << "\n"
          << "config_file: " << configfile << "\n";
#endif

    Tribots::ConfigReader vread (2);
    bool success = vread.append_from_file (configfile.c_str());
    vread.append_from_command_line (argc, argv);  // evtl. Parameter durch Kommandozeile ueberschreiben
    vread.set("rotate_log" , rotate_log);
    vread.set("add_com_user_interface",false);
    if (set_stream_interface) vread.set("user_interface_type","StreamUserInterface");
    const std::vector<std::string>& conffiles = vread.list_of_sources();
    if (!success) {
      cerr << "Fehler: konnte Konfigurationsdateien nicht vollstaendig lesen\n";
      cerr << "Konfigurationsdatei war: " << configfile << '\n';
      cerr << "Gelesen wurde aus den Dateien:\n";
      for (unsigned int i=0; i<conffiles.size(); i++)
        cerr << conffiles[i] << '\n';
      return -1;
    }
    Tribots::Journal::the_journal.set_mode (vread);
    for (unsigned int i=0; i<conffiles.size(); i++)
      JMESSAGE((std::string("tried to read from config file ")+conffiles[i]).c_str());

    std::ofstream lockfile (".robotcontrol_lock");
    if (lockfile) {
      lockfile << getpid () << std::endl;
    }

    unsigned int ui;
    if (vread.get ("random_seed", ui)>0)
      Tribots::random_seed (ui);
   

   the_world_model= new WorldModel(vread);
   the_user_interface = new UserInterface (vread, *the_world_model);

  // ggf. Anfangsposition setzen
     vector<double> dd;
     if (vread.get ("initial_position", dd)>=2) {
     if (dd.size()>=3)
         the_world_model->reset (Vec (dd[0], dd[1]),Angle::deg_angle(dd[2])-Angle::quarter);
     else
         the_world_model->reset (Vec (dd[0], dd[1]));
     }

  long int loop_time;

  unsigned long int num_cycles=0;              ///< Anzahl Iterationen

  if (vread.get ("loop_time", loop_time)<=0 || loop_time<=0) {
    JERROR("no config line \"loop_time\" found");
    throw InvalidConfigurationException ("loop_time");
  }





  int drive_vector_delay = the_world_model->get_robot_properties().drive_vector_delay;


  timestamp.update();
    Time expected_execution_time (timestamp);
    expected_execution_time.add_msec (loop_time+drive_vector_delay);
    num_cycles++;
    the_world_model->init_cycle(timestamp, expected_execution_time);




// ROS Stuff
//  ros::Publisher *rossender;
/*****************************************************/
  ros::init(argc, argv, "worldmodel");
  ros::NodeHandle n;
  ros::Subscriber sub=n.subscribe("VisibleObjects", 1,WorldModelCycle);
  ros::Subscriber sub2=n.subscribe("drivecommands", 1,setOdometry);
  ros::Publisher worldmodel_sender = n.advertise<opentribot_messages::WorldModel>("WorldModelMessages",1);
  drivecommands_sender=&worldmodel_sender;
/*****************************************************/



cout << "Worldmodel Cycle " <<num_cycles<<endl;


ros::spin();









  return 0;
}
