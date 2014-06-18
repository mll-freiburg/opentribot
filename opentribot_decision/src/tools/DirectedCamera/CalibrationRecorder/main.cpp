
#include "CalibrationRecorder.h"
#include <qapplication.h>
#include <fstream>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;


int main (int argc, char** argv) {
  if (argc<3) {
    cerr << argv[0] << " [-o Zieldatei] [-c Konfigdatei_Kameraparemeter] [-s Kamera-Section] [-h Hostname_Anzeige]\n";
    cerr << "Sammelt interaktiv Marker auf und schreibt sie in eine Markerdatei\n";
    cerr << "Kommuniziert dabei mit einem Host, der die Marker auf einem Monitor\n";
    cerr << "anzeigt, der uber eine TCP-Verbindung verbunden ist\n";
    cerr << "Auf dem externen Rechner muss das Programm 'CalibrationMarkerDisplay'\n";
    cerr << "bereits gestartet sein.\n";
    return -1;
  }
  try{
    QApplication a (argc, argv);
    ConfigReader cfg;
    cfg.add_command_line_shortcut ("o", "targetfile", true);
    cfg.add_command_line_shortcut ("c", "configfile", true);
    cfg.add_command_line_shortcut ("h", "hostname", true);
    cfg.add_command_line_shortcut ("s", "camerasection", true);
    cfg.append_from_command_line (argc, argv);
    string targetfile="";
    string configfile="";
    string hostname="localhost";
    cfg.get ("configfile", configfile);
    if (configfile.length()>0) {
      cfg.append_from_file (configfile.c_str());
      cfg.append_from_command_line (argc, argv);
    }
    cfg.get ("targetfile", targetfile);
    cfg.get ("hostname", hostname);
    CalibrationRecorder rec (cfg, hostname.c_str(), 22312);
    ostream* markerfile;
    if (targetfile.length()>0) {
      markerfile = new ofstream (targetfile.c_str());
      rec.writeMarkers (*markerfile ? *markerfile : cout);
    } else {
      rec.writeMarkers (cout);
    }
    
    rec.loop();
    (*markerfile) << flush;
  }catch(TribotsException& e){
    cerr << e.what() << endl;
  }catch(exception& e){
    cerr << e.what() << endl;
  }
}
