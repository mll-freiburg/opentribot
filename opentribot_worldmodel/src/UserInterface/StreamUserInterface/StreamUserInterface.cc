
#include <sstream>
#include <cstring>
#include <cmath>
#include <stdio.h>
#include <termios.h>
#include <iomanip>
#include "StreamUserInterface.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Structures/GameState.h"
#include "../../Player/WhiteBoard.h"
#include "../../Structures/Journal.h"

using namespace std;

namespace {

  int kbhit()
  {
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
  }
  
  void unblock_stdin(int state)
  {
    struct termios ttystate;
    
    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);
    
    if (state==1)
      {
        //turn off canonical mode
        ttystate.c_lflag &= ~ICANON;
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
      }
    else
      {
        //turn on canonical mode
        ttystate.c_lflag |= ICANON;
      }
    //set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    
  }

  

}


Tribots::StreamUserInterface::StreamUserInterface (const ConfigReader& vr, WorldModel& wm) throw () :  the_world_model (wm), single_step_mode(0), wait_for_manual_start(false), manual_start_sec (10), requestedImage(false), update_frequency (1), cycle_counter(0)
{
  // Terminalfenster vorbereiten
  update_output ();
  
  int i;
  img_no = 0;
  debug_image_filename_base = "debug_image";
  if ((vr.get ("manual_start_waiting_time", i)>0) && (i>0))
    manual_start_sec=i;
  if (vr.get ("debug_image_filename_base", debug_image_filename_base) < 0) {
    throw InvalidConfigurationException("debug_image_filename_base");
  }
  unsigned int ii=0;
  if ((vr.get ("user_interface_update_frequency", ii)>0) && ii>0 && i<=900) {
    update_frequency=ii;
  }
  unblock_stdin(1);
}

Tribots::StreamUserInterface::~StreamUserInterface () throw () {
  // Terminalfenster wieder freigeben
  unblock_stdin(0);
}

bool Tribots::StreamUserInterface::process_messages () throw () {
  if (single_step_mode>0) {
    single_step_mode--;
    if (single_step_mode==0)
      MWM.startstop (false);
  }
  if (wait_for_manual_start && (manual_start_timer.elapsed_msec()>1000*manual_start_sec)) {
    wait_for_manual_start=false;
    MWM.startstop (true);
  }

  cycle_counter++;
  if (cycle_counter<update_frequency) {
    return true;
  }
  
  cycle_counter=0;


  // Tastatur abfragen
  int  c=-1;
  //char keybuf[100];
  //int  num_keys = 0;

  //  if (kbhit()) {
  //   fgets(keybuf , 100, stdin);
  //   num_keys=strlen(keybuf);
  //  }

  if (kbhit()) {
    c = fgetc(stdin);
  }

  // std::cerr << "Read keys: " << num_keys << "\n";
  
  // if (num_keys!=1) c=-1;  // kein Zeichen oder mehrere Zeichen (vermutlich ESC-Sequenz), diese Faelle ignorieren
  // else c = keybuf[0];
  
  switch (c) {
  case ' ' : // Roboter stoppen
    MWM.startstop (false);
    single_step_mode=0;
    break;
  case 'a' : // Roboter aktivieren, aber den RefereeState nicht veraendern
    MWM.startstop (true);
    single_step_mode=0;
    break;
  case 'g' : // Roboter aktivieren und auf Refereestate FreePlay gehen
    MWM.startstop (true);
    single_step_mode=0;
    MWM.update_refbox (SIGstop);
    MWM.update_refbox (SIGstart);
    break;
  case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': // Einzelschrittmodus
    MWM.startstop (true);
    single_step_mode=static_cast<unsigned int>(c-'0');
    break;
  case 'm' : // Start in 10 Sekunden
    wait_for_manual_start=true;
    manual_start_timer.update();
    break;
  case 's' : // Seitenwechsel
    MWM.set_own_half(-MWM.get_own_half());
    break;
  case 'p' : // Selbstlokalisierung zuruecksetzen
    WorldModel::get_main_world_model().reset ();
    break;
  case 'L' : // lokalisieren auf der rechten Seite
    MWM.reset (MWM.get_own_half()*Vec(0.5*MWM.get_field_geometry().field_width+300,0), Angle::quarter+(MWM.get_own_half()<0 ? Angle::zero : Angle::half));
    break;
  case 'l' : // lokalisieren auf der linken Seite
    MWM.reset (-MWM.get_own_half()*Vec(0.5*MWM.get_field_geometry().field_width+300,0), Angle::three_quarters+(MWM.get_own_half()<0 ? Angle::zero : Angle::half));
    break;
  case 'T' : // lokalisieren auf der rechten Seite des eigenen Tors
    MWM.reset (Vec(0.5*MWM.get_field_geometry().goal_area_width+200,-0.5*MWM.get_field_geometry().field_length+MWM.get_field_geometry().goal_area_length), Angle::zero);
    break;
  case 't' : // lokalisieren auf der linken Seite des eigenen Tors
    MWM.reset (Vec(-0.5*MWM.get_field_geometry().goal_area_width-200,-0.5*MWM.get_field_geometry().field_length+MWM.get_field_geometry().goal_area_length), Angle::zero);
    break;
  case 'q': case 'Q':
    return false;   // Programm beenden
  }

  // Bildschirmansicht aktualisieren
  update_output();
  return true;
}


void Tribots::StreamUserInterface::update_output () throw () {
  /**
  std::stringstream inout;
  std::string line;
  Time current_time;
  inout << current_time << '\n';
  std::getline (inout, line);
  move (2,14); addstr(line.c_str()); addstr("    ");
  inout << "VCC: " 
        << format_double(MWM.get_robot_data(current_time).motor_vcc,4,1)
        << "[V]\n";
  std::getline (inout, line);
  addstr(line.c_str()); addstr("    ");
  move (3,16); addstr(Tribots::referee_state_names[MWM.get_game_state().refstate]);
  if (wait_for_manual_start)
    addstr ("manueller Start");
  else if (MWM.get_game_state().in_game)
    addstr (" aktiviert      ");
  else
    addstr (" deaktiviert    ");
  RobotLocation rl = MWM.get_robot_location (current_time);
  inout << '(' << format_double(rl.pos.x,6,0) << ", " <<  format_double(rl.pos.y,6,0) << ", " << format_double(rl.heading.get_deg(),3,0) << ", " << ( rl.kick ? "kick" : " -- " ) << ")\n";
  std::getline (inout, line);
  move (7,17); addstr(line.c_str());
  inout << '(' << format_double(rl.vtrans.x,5,2) << ", " << format_double(rl.vtrans.y,5,2) << ", " << format_double(rl.vrot,5,2) << ")\n";
  std::getline (inout, line);
  move (8,24); addstr(line.c_str());
  BallLocation bl = MWM.get_ball_location (current_time);
  inout << '(' << format_double(((bl.pos-rl.pos)/rl.heading).x,6,0) << ", " << format_double(((bl.pos-rl.pos)/rl.heading).y,6,0) << ", " << format_double(bl.pos.z,6,0) << ")";
  if (WhiteBoard::getTheWhiteBoard()->doPossessBall(current_time)) inout << " (own)\n"; else inout << "      \n";
  std::getline (inout, line);
  move (9,20); addstr(line.c_str());
  inout << '(' << format_double(bl.velocity.x,6,2) << ", " << format_double(bl.velocity.y,6,2) << ")\n";
  std::getline (inout, line);
  move (10,22); addstr(line.c_str());
  move (4,12); addstr(the_player.get_player_type());
  addstr(" / ");
  addstr(the_player.get_role());
  addstr ("        ");
  move (5,16);
  WorldModel& the_world_model (WorldModel::get_main_world_model ());
  if (the_world_model.get_own_half()>0)
    addstr("gelb");
  else
    addstr("blau");
  move (14,0);
  **/

  std::ostringstream out_buf;
  
  Time current_time;
  
  out_buf << "[BEGIN_CYCLE_INFO]";
  out_buf << "Time:            " << setw(10) << current_time << " \t "
	  << "VCC:  " << setw(5)  << setprecision(2) << MWM.get_robot_data(current_time).motor_vcc << "\n"
	  << "referees state:  " << Tribots::referee_state_names[MWM.get_game_state().refstate] << " \t "
    ;
  if (wait_for_manual_start)
    out_buf << "<font color=yellow>" << "manueller Start" << "<font color=black>";
  else if (MWM.get_game_state().in_game)
    out_buf << "<font color=green>" << " aktiviert " << "<font color=black>";
  else
    out_buf << "<font color=red>"   << " deaktiviert" << "<font color=black>";
  out_buf << "\n";

  RobotLocation rl = MWM.get_robot_location (current_time);

  out_buf << "RobotLocation:   (" 
	  << setw(10)  << setprecision(1) << fixed << rl.pos.x << "mm, "
	  << setw(10)  << setprecision(1) << fixed << rl.pos.y << "mm, "
	  << setw(10)  << setprecision(1) << fixed << rl.heading.get_deg() << "deg, "
	  << ") Kick: " << rl.kick
	  << "\n";

  out_buf << "RobotSpeeds:     ("
	  << setw(10)  << setprecision(2) << rl.vtrans.x << "m/s, "
	  << setw(10)  << setprecision(2)<< rl.vtrans.y << "m/s, "
	  << setw(10)  << setprecision(2)<< rl.vrot << "rad/s)\n ";

  BallLocation bl = MWM.get_ball_location (current_time);
/*
  out_buf << "rel. Ballpos:    ("
	  << setw(10)  << setprecision(1)<< ((bl.pos-rl.pos)/rl.heading).x << "mm, "
	  << setw(10)  << setprecision(1)<< ((bl.pos-rl.pos)/rl.heading).y << "mm, "
	  << setw(10)  << setprecision(1)<< bl.pos.z << "mm)"
	  << (WhiteBoard::getTheWhiteBoard()->doPossessBall(current_time) ? "<font color=green> (own) <font color=black>" : "  ")
	  << "\n";

  out_buf << "abs. Ballspeed:  ("
	  << setw(10)  << setprecision(1) << bl.velocity.x << "m/s, "
	  << setw(10)  << setprecision(1) << bl.velocity.y << "m/s, "
	  << setw(10)  << setprecision(1) << bl.velocity.z << "m/s)\n";
*/
  out_buf << "[END_CYCLE_INFO]";
  std::cout << out_buf.str() << "\n" << std::flush;
  
}


