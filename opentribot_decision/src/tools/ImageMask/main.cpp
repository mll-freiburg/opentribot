#include <qapplication.h>
#include "imagemaskmainwidget.h"
#include <string>
#include <iostream>

#include "../../ImageProcessing/Formation/IIDC.h"

using namespace Tribots;

int main( int argc, char ** argv )
{
  if (argc>=2 && std::string(argv[1])==std::string("--help")) {
    std::cerr << "Interaktives Programm zum Erzeugen der Maske,\n";
    std::cerr << "die Teile des Roboters im Bild ausblendet\n";
    std::cerr << "Aufruf: " << argv[0] << " [Config-File]\n";
    std::cerr << "wird Config-File nicht angegeben, wird auf\n";
    std::cerr << "../config_files/robotcontrol.cfg zugegriffen\n";
    return -1;
  }
  try{
    QApplication a( argc, argv );
    ImageMaskMainWidget w;
    w.show();
    a.connect( &a, SIGNAL( lastWindowClosed() ), &a, SLOT( quit() ) );
    return a.exec();
  }catch(Tribots::TribotsException& e){
    std::cerr << e.what() << '\n';
    return -1;
  }catch(std::exception& e){
    std::cerr << e.what() << '\n';
    return -1;
  }
}
