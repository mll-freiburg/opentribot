
#include "ControlGUI.h"
#include <qapplication.h>
#include <qtimer.h>
#include <iostream>

int main (int argc, char** argv) {
  try{
    QApplication a (argc, argv);
    ControlGUI w;
    qApp->setMainWidget(&w);
    w.show(); //Maximized();
    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );

    return a.exec();
  }catch(Tribots::TribotsException& e){
    std::cerr << "EXCEPTION: " << e.what() << std::endl;
    return -1;
  }catch(std::exception& e){
    std::cerr << "EXCEPTION: " << e.what() << std::endl;
    return -1;
  }
}
