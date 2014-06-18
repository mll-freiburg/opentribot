
#include <QtGui/QApplication>
#include <iostream>
#include "TribotsviewMainWidget.h"


int main (int argc, char *argv[]) {
  try{
    std::deque<std::string> logfiles;
    std::string cfgfile = "";
    if (argc>1)
      if (std::string(argv[1])=="--help" || std::string(argv[1])=="-h") {
      std::cout << "Aufruf: " << argv[0] << " [-fKonfigurationsdatei][-s][Logdatei-Praefix1 [Logdatei-Praefix2 ...]] \n";
        std::cout << "wird Logdatei-Praefix weggelassen, wird wminfo gewaehlt\n";
        std::cout << "wird Konfigurationsdatei weggelassen, wird die Feldgeometrie\n";
        std::cout << "aus dem Logfile gelesen, falls vorhanden\n";
        std::cout << "werden mehrere Logfiles angegeben, werden mehrere Roboter angezeigt\n";
        std::cout << "mit Option -s wird Synchronisation mehrerer Roboter auf der Basis der\n";
        std::cout << "Synchronisationssignale von Teamcontrol durchgefuehrt anstatt erstem Zeitstempel\n";
        return -1;
      }

    bool use_synch_signals = false;
    for (int i=1; i<argc; i++) {
      std::string arg=argv[i];
      if (arg.substr(0,2)=="-f")
        cfgfile = arg.substr (2, arg.length());
      else if (arg.substr(0,2)=="-s")
        use_synch_signals=true;
      else
        logfiles.push_back (arg);
    }
    if (logfiles.size()==0)
      logfiles.push_back (std::string ("wminfo"));

    QApplication app (argc,argv);

    TribotsTools::TribotsviewMainWidget tv (NULL);
    tv.init_field_and_streams (logfiles, cfgfile, use_synch_signals);
    tv.show();
    app.connect( &app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()) );

    return app.exec ();
  }catch(std::exception& e){
    std::cerr << e.what() << std::endl;
    return -1;
  }
}



