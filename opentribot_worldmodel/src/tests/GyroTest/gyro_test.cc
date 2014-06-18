
#include "../../Robot/CompassGrabbingThread.h"
#include <iostream>

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc!=2) {
    cerr << argv[0] << " Kompass-Device (/dev/ttyUSBx)\n";
    cerr << " kommuniziert uber RS-232 mit 3DM-G\n";
    return -1;
  }
  try{
  CompassGrabbingThread grab (argv[1],200);
  char c=' ';
  grab.start();
  while (c!='q') {
//    cin >> c;
    CompassGrabbingThread::GyroDataTime data;
    do {
      data=grab.getValue();
      if (data.valid)
        cout << data.timestamp.get_msec() << '\t' << data.gyro.vrot << endl;
    }while (data.valid);
  }
  grab.cancel();
  grab.waitForExit();
  return 0;
  }catch(exception& e){
    cerr << e.what() << endl;
    return -1;
  }
}
