
#include "DistanceCalibrationCollectionWidget.h"
#include "../../Structures/Journal.h"
#include <qapplication.h>
#include <qtimer.h>
#include <iostream>

int main (int argc, char** argv) {
  try{
    Tribots::Journal::the_journal.set_stream_mode (std::cerr);
    QApplication a (argc, argv);
    DistanceCalibrationCollectionWidget w;
    qApp->setMainWidget(&w);
    w.showMaximized();
    QTimer t;
    t.start (33);
    a.connect( &a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
    a.connect( &t, SIGNAL(timeout()), &w, SLOT(loop()) );

    return a.exec();
  }catch(Tribots::TribotsException& e){
    std::cerr << "EXCEPTION: " << e.what() << std::endl;
    return -1;
  }catch(std::exception& e){
    std::cerr << "EXCEPTION: " << e.what() << std::endl;
    return -1;
  }
}
