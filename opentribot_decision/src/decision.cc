
#include "Structures/Journal.h"
#include "Fundamental/random.h"
#include <string>
#include <fstream>
#include <sys/types.h>
#include <unistd.h>
#include "WorldModel/Types/RosListenerWorldModel.h"
#include "Player/Player.h"
#include "Player/WhiteBoard.h"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "opentribot_messages/WorldModel.h"
#include "opentribot_messages/TribotDrive.h"
#include "opentribot_messages/TeamSignal.h"
#include "UserInterface/UserInterface.h"




using namespace std;
using namespace Tribots;

//#define DEBUGCMDLINEARG 1
Player * the_player;
//WorldModel * the_world_model;
WorldModel * the_world_model;
ros::Publisher * global_drivecommand_sender;
UserInterface * the_user_interface;






int main (int argc, char** argv) {
ros::init(argc, argv, "Decision");

   	Time timestamp;


 

  // recover previous Worldmodel position
  bool auto_restart=true;
  /** rotate_log: decides if time information is appended to logfile
   * names so logfiles are not lost on multiple calls of the program
   * (Communicated using configreader, read in AddWriteWorldModel)
   **/
  bool rotate_log = true;
  bool set_stream_interface = false;
  std::string configfile = ("config_files/robotcontrol.cfg");

  // Command line argument parsing:
  Tribots::ConfigReader cfg_cl (0);
  cfg_cl.add_command_line_shortcut ("h","help",false);
  cfg_cl.add_command_line_shortcut ("restart","restart",false);
  cfg_cl.add_command_line_shortcut ("norestart","norestart",false);
  cfg_cl.add_command_line_shortcut ("l","no_rotate_log",false);
  cfg_cl.add_command_line_shortcut ("no_rotate_log","no_rotate_log",false);
  cfg_cl.add_command_line_shortcut ("stream_interface","stream_interface",false);
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
   
    
    
    vread.set("world_model_type","RosListenerWorldModel");    
    
    
    the_world_model= new WorldModel(vread);
    the_user_interface = new UserInterface (vread, *the_world_model);


    timestamp.update();
    Time expected_execution_time (timestamp);
    expected_execution_time.add_msec (20+80);

    the_world_model->init_cycle(timestamp, expected_execution_time);
    
   the_player = new Player (vread);




  long int loop_time;

  unsigned long int num_cycles=0;              ///< Anzahl Iterationen

  if (vread.get ("loop_time", loop_time)<=0 || loop_time<=0) {
    JERROR("no config line \"loop_time\" found");
    throw InvalidConfigurationException ("loop_time");
  }



num_cycles++;
ros::NodeHandle n;

ros::Publisher drivecommand_sender = n.advertise<opentribot_messages::TribotDrive>("drivecommands",1);
    global_drivecommand_sender=&drivecommand_sender;


int stopcycles=0;

while (ros::ok()){
Time ex_time;
//ex_time.add_msec(30);
//ros::spinOnce();
ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(10000000));


  // stelle Startzeit des Schleifendurchlaufs fest (wegen Taktung)
Time timestamp;    
timestamp.update();
Time expected_execution_time (timestamp);
expected_execution_time.add_msec (loop_time+MWM.get_robot_properties().drive_vector_delay);
//num_cycles++;
the_world_model->init_cycle(timestamp, expected_execution_time);

//cout << "Possessball"<<WBOARD->doPossessBall(Time())<<endl;
the_world_model->update();






  
  
  //WBOARD->resetPossessBall();

//cout <<"Velocity "<< MWM.get_ball_location(timestamp).velocity<<endl;



DriveVector dv = the_player->process_drive_vector (ex_time);

opentribot_messages::TribotDrive dmsg;

if (the_world_model->get_game_state().in_game) {  // ueberpruefen, ob der Spieler tatsaechlich im Spiel ist
stopcycles=0;
opentribot_messages::TribotDrive dmsg;

//dv.vtrans=Vec(0,0);

dmsg.sender="Decision";
dmsg.vtransx=dv.vtrans.x;
dmsg.vtransy=dv.vtrans.y;
dmsg.vrot=dv.vrot;
dmsg.kick=dv.kick;
dmsg.kicklength=dv.klength;
if (dmsg.kick==0)dmsg.kicklength=0;
       
        global_drivecommand_sender->publish(dmsg);
        MWM.set_drive_vector(dv, timestamp);


}
else{
	dmsg.sender="Decision";
	dmsg.vtransx=0;
	dmsg.vtransy=0;
	dmsg.vrot=0;
	dmsg.kick=0;
	dmsg.kicklength=0;
	if (stopcycles<5)global_drivecommand_sender->publish(dmsg);
	stopcycles++;

}
the_user_interface->process_messages();


}



cout << "ending decision"<<endl;

return 0;
}
