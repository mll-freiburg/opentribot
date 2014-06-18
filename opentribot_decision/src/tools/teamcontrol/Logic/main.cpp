
#include <qapplication.h>
#include <iostream>
#include "../Widgets/TeamcontrolMainWidget.h"
#include "../../../Structures/Journal.h"


int main (int argc, char *argv[]) {
  try{
    Tribots::ConfigReader cfg;
    std::string cfg_file = "../config_files/teamcontrol.cfg";
    bool nolog=false;
    for (int i=1; i<argc; i++) {
      std::string arg = argv[i];
      if (arg=="--help" || arg=="-h") {
        std::cout << "Aufruf: " << argv[0] << " [Konfigurationsdatei|-s] [-nolog]\n";
        std::cout << "Fehlt die Konfigurationsdatei, wird ../config_files/teamcontrol.cfg verwendet\n";
        std::cout << "Bei Angabe von -s wird die Datei ../config_files/teamcontrol_simulator.cfg geladen\n";
        std::cout << "Bei Angabe von -nolog werden keine Coachlog-Dateien geschrieben\n";
        return -1;
      } else if (arg=="-s") {
        cfg_file = "../config_files/teamcontrol_simulator.cfg";
      } else if (arg=="-nolog") {
        nolog=true;
      } else {
        cfg_file = argv[1];
      }
    }
    cfg.append_from_file (cfg_file.c_str());
    if (nolog)
      cfg.set ("nolog", true);
    QApplication app (argc,argv);

    Tribots::Journal::the_journal.set_stream_mode (std::cerr);

    TeamcontrolMainWidget tcw (0,"main window");
    app.setMainWidget(&tcw);
    tcw.init (cfg);
    tcw.show();

    return app.exec ();
  }catch (Tribots::TribotsException& e) {
    std::cerr << e.what() << '\n';
    return -1;
  }catch (std::exception& e) {
    std::cerr << e.what() << '\n';
    return -1;
  }
}
