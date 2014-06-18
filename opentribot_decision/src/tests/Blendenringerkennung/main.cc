
#include "../../ImageProcessing/Formation/PPMIO.h"
#include "../../ImageProcessing/Formation/RGBImage.h"
#include "../../Fundamental/Time.h"
#include "../../ImageProcessing/Calibration/centerRingOperation.h"
#include <iostream>
#include <fstream>

using namespace Tribots;
using namespace std;

int main (int argc, char** argv) {
  if (argc<3) {
    cerr << argv[0] << " <PPM-Zieldatei> <PPM-Quelldatei>\n";
    return -1;
  }
  try{
    ifstream src (argv[2]);
    ofstream dest (argv[1]);

    PPMIO io;
    ImageBuffer* sim = io.read (NULL, src);
    if (!sim) {
      cerr << "Kein Bild gelesen. Fehler.\n";
      return -1;
    }
    RGBImage img (*sim);

    RGBImage tgimg (sim->width, sim->height);

    Time start;
    double ccx, ccy, ccr1, ccr2;
    bool succ = findCenterRing (ccx, ccy, ccr1, ccr2, tgimg, img, true);
    cerr << "succ = " << succ << ' ' << ccx << ' ' << ccy << ' ' << ccr1 << ' ' << ccr2 << endl;
    cerr << "time = " << static_cast<double>(start.elapsed_usec())*1e-3 << " ms\n";


    io.write (tgimg.getImageBuffer(), dest);
    dest << flush;
    cerr << "Zieldatei " << argv[1] << " geschrieben\n";
  }catch(TribotsException& e){
    cerr << "TException: " << e.what() << endl;
    return -1;
  }catch(exception& e){
    cerr << "Exception: " << e.what() << endl;
    return -1;
  }
  return 0;
}
