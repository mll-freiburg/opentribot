
#include <qapplication.h>
#include <iostream>
#include <deque>
#include "TribotsviewMainWidget.h"


int main (int argc, char *argv[]) {
  try{
    std::deque<std::string> logfiles;
    std::string cfgfile = "";
    if (argc>1)
      if (std::string(argv[1])=="--help" || std::string(argv[1])=="-h") {
      std::cout << "Aufruf: " << argv[0] << " [-h][-f Feldgeometrie][-s][-c][-a][Logdatei1 [Logdatei2 ...]]\n";
      std::cout << "mit:\n";
      std::cout << "  Logdatei: die Logdatei (mit oder ohne Suffix), aus der gelesen werden soll.\n";
      std::cout << "      Wird keine Logdatei angegeben, wird aus wminfo.log gelesen\n";
      std::cout << "      Werden mehrere Logfiles angegeben, werden mehrere Roboter visualisiert.\n";
      std::cout << "      Zwischen den Robotern kann mit den F-Tasten umgeschaltet werden\n";
      std::cout << "  Feldgeometrie: eine Konfigurationsdatei, die die Feldgeometrie enthaelt.\n";
      std::cout << "      Wird keine Feldgeometrie explizit angegeben, wird die Feldgeometrie aus\n";
      std::cout << "      dem Logfile gelesen.\n";
      std::cout << "  -s: legt fest, dass die Synchronisation mehrere Roboter mit Hilfe der\n";
      std::cout << "      Synchronisationssignale des Teamcontrols erfolgen soll. Andernfalls\n";
      std::cout << "      erfolgt die Synchronisation anhand der Systemzeiten.\n";
      std::cout << "  -c: legt fest, dass bei der Darstellung des Feldes ein gelbes und ein\n";
      std::cout << "      blaues Tor gezeichnet werden an Stelle eines Richtungspfeils.\n";
      std::cout << "  -a: verhindert, dass deaktivierte Roboter dunkelgrau erscheinen.\n";
      std::cout << "  -h: zeigt diese Hilfe an.\n";
      return -1;
    }

    bool use_synch_signals = false;
    bool use_colored_goals = false;
    bool use_attributes=true;
    for (int i=1; i<argc; i++) {
      std::string arg=argv[i];
      if (arg.substr(0,2)=="-f" && i+1<argc)
        cfgfile = argv[++i];
      else if (arg.substr(0,2)=="-s")
        use_synch_signals=true;
      else if (arg.substr(0,2)=="-c")
        use_colored_goals=true;
      else if (arg.substr(0,2)=="-a")
        use_attributes=false;
      else
        logfiles.push_back (arg);
    }
    if (logfiles.size()==0)
      logfiles.push_back (std::string ("wminfo"));

    QApplication app (argc,argv);

    TribotsviewMainWidget tv (0,"main window");
    tv.init_field_and_streams (logfiles, cfgfile, use_synch_signals, use_colored_goals, use_attributes);

    app.setMainWidget(&tv);
    tv.show();

    return app.exec ();
  }catch(std::exception& e){
    std::cerr << e.what() << std::endl;
    return -1;
  }
}



