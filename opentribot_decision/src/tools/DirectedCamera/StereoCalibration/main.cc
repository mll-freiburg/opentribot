
#include "StereoCalibration.h"
#include <iostream>
#include <fstream>
#include <qapplication.h>
#include "../../../Structures/Journal.h"

using namespace Tribots;
using namespace std;

int main (int argc, char** argv) {
  if (argc<3) {
    cerr << argv[0] << " Konfigurationsfile KamerasectionOmnicam <PPM-Quelldatei1> <PPM-Quelldatei2> [<PPM-Quelldatei1> <PPM-Quelldatei2> [...]]\n";
    cerr << "Das Ergebnis wird nach stdout geschrieben\n";
    return -1;
  }
  try{

    QApplication a (argc, argv);
    StereoCalibration calib;
    a.setMainWidget (&calib);
    calib.writeCorrespondences (cout);
    calib.show ();
    Tribots::Journal::the_journal.set_stream_mode (std::cerr);

    int ret = a.exec ();
    cout << flush;
    return ret;
  }catch(TribotsException& e){
    cerr << "TException: " << e.what() << endl;
    return -1;
  }catch(exception& e){
    cerr << "Exception: " << e.what() << endl;
    return -1;
  }
  return 0;
}
