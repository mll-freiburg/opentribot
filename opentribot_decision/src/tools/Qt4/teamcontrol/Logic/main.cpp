
#include <QtGui/QApplication>
#include <iostream>
#include "../Widgets/TeamcontrolMainWidget.h"
#include "../../../../Structures/Journal.h"

using namespace TribotsTools;

int main (int argc, char *argv[]) {
  try{
    Tribots::ConfigReader cfg;
    std::string cfg_file = "../config_files/teamcontrol.cfg";
    if (argc>1) {
      if (std::string(argv[1])=="--help" || std::string(argv[1])=="-h") {
        std::cout << "Aufruf: " << argv[0] << " [Konfigurationsdatei]\n";
        std::cout << "Fehlt die Konfigurationsdatei, wird ../config_files/teamcontrol.cfg verwendet\n";
        return -1;
      } else {
        cfg_file = argv[1];
      }
    }
    cfg.append_from_file (cfg_file.c_str());
    QApplication app (argc,argv);

    Tribots::Journal::the_journal.set_mode (cfg);

    TeamcontrolMainWidget tcw (0);
    tcw.init (cfg);
    app.connect( &tcw, SIGNAL(widgetClosed()), &app, SLOT(quit()) );
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
