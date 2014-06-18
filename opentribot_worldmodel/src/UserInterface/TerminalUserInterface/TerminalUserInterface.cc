
#include <sstream>
#include <cstring>
#include <cmath>
#include "TerminalUserInterface.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Structures/GameState.h"
#include "../../Player/WhiteBoard.h"
#include "../../Structures/Journal.h"
#include "../../ImageProcessing/Formation/PPMIO.h"

using namespace std;

namespace {

  // formatierte double-Werte
  string format_double (double d, unsigned int flen, unsigned int prec) {
    std::stringstream inout;
    inout.precision (prec);
    inout.setf(ios_base::fixed,ios_base::floatfield);
    inout << d;
    string res;
    std::getline (inout, res);
    int n=flen-res.length();
    while (n>0) {
      res=string(" ")+res;
      n--;
    }
    return res;
  }
   
} 


Tribots::TerminalUserInterface::TerminalUserInterface (const ConfigReader& vr, Player& pl, Vision& ip, WorldModel& wm, Robot& rb) throw () : the_player(pl), the_vision(ip), the_world_model (wm), the_robot (rb), single_step_mode(0), wait_for_manual_start(false), manual_start_sec (10), requestedImage(false), update_frequency (10), cycle_counter(0)
{
  // Terminalfenster vorbereiten
  window = initscr();
  cbreak();
  noecho();
  nodelay(window,true);
  init_window ();
  update_window ();
  
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
}

Tribots::TerminalUserInterface::~TerminalUserInterface () throw () {
  // Terminalfenster wieder freigeben
  endwin();
}

bool Tribots::TerminalUserInterface::process_messages () throw () {
  if (single_step_mode>0) {
    single_step_mode--;
    if (single_step_mode==0)
      MWM.startstop (false);
  }
  if (wait_for_manual_start && (manual_start_timer.elapsed_msec()>1000*manual_start_sec)) {
    wait_for_manual_start=false;
    MWM.startstop (true);
  }
  if (requestedImage) {
    try {
      for (int i=0; i < the_vision.get_num_sources(); i++) {
        if (the_vision.is_image_available(i)) {
          const Image* img = the_vision.get_image(i);
          save_image(img, i, img_no);
          the_vision.free_image(i);
          requestedImage = false;
        }
      }
      img_no++;
    } catch (TribotsException& e) {
      JERROR(e.what());
    }
  }

  cycle_counter++;
  if (cycle_counter<update_frequency) {
    return true;
  }
  
  cycle_counter=0;
  // Tastatur abfragen
  int c=-1;
  unsigned int num_keys=0;
  while (true) {
    int d = getch ();
    if (d==-1) break;  // keine weiteren Zeichen
    num_keys++;
    c=d;
  };
  if (num_keys!=1) c=-1;  // kein Zeichen oder mehrere Zeichen (vermutlich ESC-Sequenz), diese Faelle ignorieren
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
  case 'i':
    if (!requestedImage) {
      try {
        for (int i=0; i < the_vision.get_num_sources(); i++) {
          the_vision.request_image_raw(i);
          requestedImage = true;
        }
      } catch (TribotsException& e) {
        JERROR(e.what());
        requestedImage = false;
      }
    }
    break;
  case 'I':
    if (!requestedImage) {
      try {
        for (int i=0; i < the_vision.get_num_sources(); i++) {
          the_vision.request_image_processed(i);
          requestedImage = true;
        }
      } catch (TribotsException& e) {
        JERROR(e.what());
        requestedImage = false;
      }
    }
    break;
  case 'm' : // Start in 10 Sekunden
    wait_for_manual_start=true;
    manual_start_timer.update();
    break;
  case 's' : // Seitenwechsel
    MWM.set_own_half(-MWM.get_own_half());
    break;
  case 'f' : // Bildschirm refresh
    init_window();
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
  case 'G' : // Spielertyp Goalie setzen
    change_player ("Goalie");
    break;
  case 'F' : // Spielertyp
    change_player ("Feldspieler");
    break;
  case 'H' : // Spielertyp Feldspieler07  (H=Hannover)
    change_player ("Feldspieler07");
    break;
  case 'J' : // Spielertyp Feldspieler07  (H=Hannover)
    change_player ("JoystickPlayerUDP");
    break;
  case 'q': case 'Q':
    return false;   // Programm beenden
  }

  // Bildschirmansicht aktualisieren
  update_window();
  return true;
}


void Tribots::TerminalUserInterface::init_window () throw () {
  clear();
  addstr ("TRIBOTS - ROBOTCONTROL");
  move (2,0); addstr("Programmzeit: ");
  move (3,0); addstr("Spielsituation: ");
  move (4,0); addstr("Spielertyp: ");
  move (5,0); addstr("Eigene Haelfte: "); 
  move (7,0); addstr("Roboterposition: ");
  move (8,0); addstr("Robotergeschwindigkeit: ");
  move (9,0); addstr("Ballposition (rel): ");
  move (10,0); addstr("Ballgeschwindigkeit: ");
  move (12,0); addstr(the_vision.get_vision_type()); addstr(", "); addstr(the_world_model.get_world_model_type()); addstr(", ");
  addstr(the_player.get_player_type()); addstr(", "); addstr(the_robot.get_robot_type());
  move (13,0); addstr("-----------------------------------------------------");
}  

void Tribots::TerminalUserInterface::update_window () throw () {
  std::stringstream inout;
  std::string line;
  Time current_time;
  inout << format_double (static_cast<double>(current_time.get_msec())/1000.0, 0, 3) << '\n';
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
}


void Tribots::TerminalUserInterface::save_image(const Image* img, int source, int no) const throw()
{
  stringstream str;
  str << debug_image_filename_base << "_" << source << "_" << no << ".ppm";
  try {
    ImageIO* io = new PPMIO();
    io->write(img->getImageBuffer(), str.str().c_str());
    delete io;
  } catch (TribotsException& e) {
    JERROR (e.what());
  }
}


void Tribots::TerminalUserInterface::change_player(const char* newpl) const throw () {
  if (std::string(newpl)==std::string(the_player.get_player_type())) {
    // Rolle weiterschalten
    unsigned int cindex=0;
    for (unsigned int i=0; i<the_player.get_list_of_roles().size(); i++)
      if (the_player.get_list_of_roles()[i]==std::string(the_player.get_role()))
        cindex=i;
    the_player.set_role (the_player.get_list_of_roles()[(cindex+1)%the_player.get_list_of_roles().size()].c_str());
  } else {
    // Spielertyp setzen
    the_player.change_player_type (newpl);
  }
}
