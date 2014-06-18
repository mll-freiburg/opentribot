
#include <sstream>
#include <cstring>
#include <cmath>
#include <iomanip>
#include "CliUserInterface.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Structures/GameState.h"
#include "../../Player/WhiteBoard.h"
#include "../../Structures/Journal.h"


using namespace std;


#include <stdio.h>
#include <termios.h>
#include <unistd.h>
    

#define CONSOLE_RESET  	(char)27<<"[0;0m"

#define CONSOLE_BLACK   (char)27<<"[1;30m"
#define CONSOLE_RED   	(char)27<<"[1;31m"
#define CONSOLE_GREEN   (char)27<<"[1;32m"
#define CONSOLE_YELLOW  (char)27<<"[1;33m"
#define CONSOLE_BLUE   	(char)27<<"[1;34m"
#define CONSOLE_MAGENTA (char)27<<"[1;35m"
#define CONSOLE_CYAN   	(char)27<<"[1;36m"
#define CONSOLE_WHITE   (char)27<<"[1;37m"


int getc(){
	
  struct termios to, t;

    tcgetattr (1, &t);
    // saving termios struct
      to = t;
      //toggling canonical mode and echo mode 
        t.c_lflag &= ~(ICANON | ECHO);

          tcsetattr (1, 0, &t);
            char c;
              c=getchar();

                tcsetattr (1, 0, &to);
                  return c;

                  }









namespace {

  // formatierte double-Werte
  string format_double (double d, unsigned int flen, unsigned int prec) {
    std::stringstream inout;
    inout.precision (prec);
    inout.setf(ios_base::fixed,ios_base::floatfield);
    double d2=d;
    if (fabs(d2)<0.001f) d2=-0.001f;
    if (fabs(d2)>-0.001f) d2=0.001f;
    string res;
    std::getline (inout, res);
    int n=flen-res.length();
    inout << d;
    if (d2>0) res=" "+res;
     while (n>0) {
      res=string(" ")+res;
      n--;
    }
    return res;
  }
   
} 


Tribots::CliUserInterface::CliUserInterface (const ConfigReader& vr, WorldModel& wm) throw () : the_world_model (wm),  single_step_mode(0), wait_for_manual_start(false), manual_start_sec (10), requestedImage(false), update_frequency (1), cycle_counter(0)
{
 // cbreak();
  //noecho();
  //nodelay(window,true);
 framecounter=0;
 update_frequency_mode=0;
update_frequency_redraw=10; 
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

Tribots::CliUserInterface::~CliUserInterface () throw () {
}

bool Tribots::CliUserInterface::process_messages () throw () {
  Time current_time;
//cout << "Currenttime Start of Userinterface "<<current_time.get_msec()<<endl;
 framecounter++;
if (framecounter%10==0)
{ 
   fps=1000*10/(current_time.get_msec()-time_10framesago);

	time_10framesago=current_time.get_msec();
  
}
 

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
  int c=-1;
  unsigned int num_keys=0;
    c=readKey_Select(); 
    int chomp=1;
    while(chomp>0)chomp=readKey_Select();

 // if (num_keys!=1) c=-1;  // kein Zeichen oder mehrere Zeichen (vermutlich ESC-Sequenz), diese Faelle ignorieren
  switch (c) {
  case ' ' : // Roboter stoppen
     //cout << "Stopping Robot!!!"<<endl;
     MWM.startstop (false);
    single_step_mode=0;
    break;
  case 'a' : // Roboter aktivieren, aber den RefereeState nicht veraendern
    MWM.startstop (true);
    single_step_mode=0;
    break;
  case 'g' : // Roboter aktivieren und auf Refereestate FreePlay gehen
   // cout <<"Starting Robot..."<<endl;
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
      cout << "Manual Start in 10 Seconds"<<endl;
    wait_for_manual_start=true;
    manual_start_timer.update();
    break;
  case 'd' : // Anzeige langsam
    cout << "changing Refresh frequency"<<endl;
    update_frequency_mode+=1;
    update_frequency_mode=update_frequency_mode%3;
    if (update_frequency_mode==0)update_frequency_redraw=1;
    if (update_frequency_mode==1)update_frequency_redraw=10;
    if (update_frequency_mode==2)update_frequency_redraw=1000;
    break;
  case 's' : // Seitenwechsel
     cout << "Side change"<<endl;
    MWM.set_own_half(-MWM.get_own_half());
    break;
  case 'f' : // Bildschirm refresh
     cout << "Screen Refresh"<<endl;
    break;
  case 'p' : // Selbstlokalisierung zuruecksetzen
     cout << "Reset self-localization"<<endl;
    WorldModel::get_main_world_model().reset ();
    break;
  case 'L' : // lokalisieren auf der rechten Seite
     cout << "Localize on Right side"<<endl;
    WorldModel::get_main_world_model().reset ();
    MWM.reset (MWM.get_own_half()*Vec(0.5*MWM.get_field_geometry().field_width+300,0), Angle::quarter+(MWM.get_own_half()<0 ? Angle::zero : Angle::half));
    break;
  case 'l' : // lokalisieren auf der linken Seite
     cout << "Localize on Left side"<<endl;
    MWM.reset (-MWM.get_own_half()*Vec(0.5*MWM.get_field_geometry().field_width+300,0), Angle::three_quarters+(MWM.get_own_half()<0 ? Angle::zero : Angle::half));
    break;
  case 'T' : // lokalisieren auf der rechten Seite des eigenen Tors
     cout << "Localize on right Side at Goal"<<endl;
    MWM.reset (Vec(0.5*MWM.get_field_geometry().goal_area_width+200,-0.5*MWM.get_field_geometry().field_length+MWM.get_field_geometry().goal_area_length), Angle::zero);
    break;
  case 't' : // lokalisieren auf der linken Seite des eigenen Tors
     cout << "Localize on right Side at own GOal"<<endl;
    MWM.reset (Vec(-0.5*MWM.get_field_geometry().goal_area_width-200,-0.5*MWM.get_field_geometry().field_length+MWM.get_field_geometry().goal_area_length), Angle::zero);
    break;
  }

  // Bildschirmansicht aktualisieren
  if (framecounter%update_frequency_redraw==0)update_window();
  return true;
}


void Tribots::CliUserInterface::init_window () throw () {
}  

void Tribots::CliUserInterface::update_window () throw () {
  std::string line;
  
  Time current_time;
  Time robotdata_time; // wirdvom roboter ausgefuellt
  
//cout <<"Current_time in update window"<< current_time.get_msec() <<endl;
  cout << CONSOLE_BLUE <<setw(4)<<noskipws<<fixed<<setfill(' ')<<setprecision(1)<< (double)(current_time.get_msec())/1000.0, 4,0 ;
  cout << "DECISION|";
  cout << fps<<"fps|";
  //cout <<"vor MWMCurrent_time in update window"<< current_time.get_msec() <<endl;
//  cout <<"Current_time in update window"<< current_time.get_msec() <<endl;

  cout <<"|"<<(Tribots::referee_state_names [MWM.get_game_state().refstate]);
  if (wait_for_manual_start)
    cout<< (" |manualstart");
  else if (MWM.get_game_state().in_game)
    
    cout<<"|" << CONSOLE_GREEN <<("!GO!");
    
  else
    cout<<"|"<< CONSOLE_RED<< ("STOP");
  
  cout<< CONSOLE_WHITE;
  RobotLocation rl = MWM.get_robot_location (current_time);
  cout <<CONSOLE_GREEN<< '|' <<setw(4)<<setfill(' ') << rl.pos.x << "," <<setw(4)<<setfill(' ')<<  rl.pos.y << "," << setw(6)<<setfill(' ')<<rl.heading.get_deg() << "," << ( rl.kick ? "kick" : "-" ) << ")";
  cout <<CONSOLE_MAGENTA<< '|'<< setw(4)<<setfill(' ') <<rl.vtrans.x << "," << setw(4)<<setfill(' ') <<rl.vtrans.y << "," << setw(4)<<setfill(' ') <<rl.vrot << ")";
  BallLocation bl = MWM.get_ball_location (current_time);cout << "currenttime "<< current_time.get_msec();
  BallRelative blr= MWM.get_ball_relative ();
  cout << '|' <<CONSOLE_RED<< setw(4)<<setfill(' ') <<bl.velocity.x << "," << setw(4)<<setfill(' ') <<bl.velocity.y << "," << setw(4)<<setfill(' ') ;
  cout << '|' <<CONSOLE_RED<< setw(4)<<setfill(' ') <<blr.pos.x << "," << setw(4)<<setfill(' ') <<blr.pos.y << "," << setw(4)<<")";
  //cout << '|' <<CONSOLE_RED<< setw(4)<<setfill(' ') <<bl.pos.x << "," << setw(4)<<setfill(' ') <<bl.pos.y<< "," << setw(4)<<setfill(' ') <<bl.pos.z<< ")";
  if (WhiteBoard::getTheWhiteBoard()->doPossessBall(current_time)) 
		cout <<"|"<< CONSOLE_RED << "own"; 
	else
		cout <<"|"<< CONSOLE_WHITE<< "---";
  
  cout << '(' << setw(4)<<setfill(' ') <<bl.velocity.x << "," << setw(4)<<setfill(' ') <<bl.velocity.y << ")";
  cout<<("/");
  WorldModel& the_world_model (WorldModel::get_main_world_model ());
  if (the_world_model.get_own_half()>0)
    cout <<"| "<< CONSOLE_YELLOW <<"gelb";
  else
    cout <<"| "<< CONSOLE_BLUE <<"blau";
 

 cout << CONSOLE_RESET <<"\n";
}







char Tribots::CliUserInterface::readKey_Select() {


  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  int max_fd_num = 0;
     int stdin_fd; 
	FD_ZERO (&rfds);
//    FD_SET (js->joystick_fd, &rfds);
      stdin_fd = fileno (stdin);
      FD_SET (stdin_fd, &rfds);
  

    struct termios to, t;

    tcgetattr (1, &t);
    // saving termios struct
     to = t;
      //toggling canonical mode and echo mode 
     t.c_lflag &= ~(ICANON );
     tcsetattr (1, 0, &t);

           char c=-1;

      int retval = select (max_fd_num + 1, &rfds, NULL, NULL, &tv);

      if (FD_ISSET (stdin_fd, &rfds))	// Tastatureingabe;
	{
	  char eingabe[100];

	  ssize_t r_bytes;	// number of bytes read into the buffer
            c=getchar();
       	cout <<"USER INPUT "<< endl;  
	
	}
	  tcsetattr (1, 0, &to);

return c;

}

