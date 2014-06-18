
#include "../ControlLoop/ControlLoop.h"
#include "../Structures/Journal.h"
#include "../Fundamental/random.h"
#include <string>
#include <fstream>
#include <sys/types.h>
#include <unistd.h>

#include "../Audio/WavePlay.h"


using namespace std;
using namespace Tribots;

//#define DEBUGCMDLINEARG 1

namespace {

  void help_text (const char* pname) {
    cerr << "Aufruf: \n"
         << pname << " [-h|--help] \n"
         << pname << " [-restart|--restart][-l|-no_rotate_log|--no_rotate_log][Konfigurationsdatei]\n"
         << "wird keine Konfigurationsdatei angegeben, wird automatisch\n"
         << "../config_files/robotcontrol.cfg verwendet\n"
         << " Optionen: \n"
         << "-h | --help : diese Zeilen \n"
         << "-restart | --restart : auto restart\n"
         << "-l | -no_rotate_log | --no_rotate_log : keine Zeitinfo in Logfile-Namen\n";
  }

}


int main (int argc, const char** argv) {
//  WavePlay::getInstance()->startPlaying(3);

  // recover previous Worldmodel position
  bool auto_restart=true;
  /** rotate_log: decides if time information is appended to logfile
   * names so logfiles are not lost on multiple calls of the program
   * (Communicated using configreader, read in AddWriteWorldModel)
   **/
  bool rotate_log = true;
  bool set_stream_interface = false;
  std::string configfile = ("../config_files/robotcontrol.cfg");

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
  
  if (cfg_cl.get("norestart",b))
    auto_restart=!b;
  if (cfg_cl.get("restart",b))
    auto_restart=b;
  if (cfg_cl.get("no_rotate_log",b))
    rotate_log=!b;
  if (cfg_cl.get("help",b)) {
    help_text(argv[0]);
    return -1;
  }
  if (cfg_cl.get("stream_interface",b)) set_stream_interface=true;
  if (cfg_cl.get("ConfigReader::unknown_argument_1", s))
    configfile=s;

#ifdef DEBUGCMDLINEARG
  std::cout << "auto_restart: " << auto_restart << "\n"
          << "rotate_log: " << rotate_log << "\n"
          << "config_file: " << configfile << "\n";
#endif

  try{
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

    Tribots::ControlLoop the_control_loop (vread, auto_restart);
    std::ofstream lockfile (".robotcontrol_lock");
    if (lockfile) {
      lockfile << getpid () << std::endl;
    }
    the_control_loop.loop();

    unsigned int ui;
    if (vread.get ("random_seed", ui)>0)
      Tribots::random_seed (ui);

  }catch(Tribots::TribotsException& e){
    cerr << e.what() << "\n\r" << std::flush;
    cerr << "consult journal for more information\n";
    Tribots::Journal::the_journal.error (__FILE__, __LINE__, e.what());
    for (unsigned int i=0; i<e.backtrace().size(); i++)
      Tribots::Journal::the_journal.message (e.backtrace()[i].c_str());
    Tribots::Journal::the_journal.flush();
    return -1;
  }catch(std::exception& e){ 
    cerr << e.what() << "\n\r" << std::flush;
    Tribots::Journal::the_journal.error (__FILE__, __LINE__, e.what());
    Tribots::Journal::the_journal.flush();
    return -1;
  }
  return 0;
}
