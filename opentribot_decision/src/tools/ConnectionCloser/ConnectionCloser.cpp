
#include "../../Communication/TribotsUDPCommunication.h"
#include "../../Structures/Journal.h"
#include <iostream>
#include <cstdlib>
#include <string>

using namespace Tribots;
using namespace std;

int main (int argc, char** argv) {
  if (argc>1 && (string(argv[1])=="-h" || string(argv[1])=="--help")) {
    cerr << "Programm, um die offene Kommunikation von unbekannten Teamcontrols vom Roboter\n";
    cerr << "aus zu schliessen. Wartet 5 Sekunden und sendet allen Teamcontrol ein\n";
    cerr << "'bye'-Signal, die in dieser Zeit an den Roboter senden.\n";
    cerr << "Aufruf: " << argv[0] << " [Port=59361]\n";
    return -1;
  }

  Tribots::Journal::the_journal.set_stream_mode (std::cerr);
  const unsigned int port = (argc>1 ? atoi(argv[1]) : 59361);
  cout << "Listening on port " << port << endl;
  TribotsUDPCommunication comm;
  comm.init_as_server (port);
  unsigned int iter=0;
  while (iter<50) {
    iter++;
    unsigned int num=comm.receive ();
    if (num>0) {
      unsigned int addr = comm.partner_address().sin_addr.s_addr;
      cout << "receiving " << num << " packets from " << (addr&255) << '.' << ((addr>>8)&255) << '.' << ((addr>>16)&255) << '.' << ((addr>>24)&255) << ". Thank you and good bye." << endl;
      comm.putBye();
      comm.send();
      iter=0;
    }
    usleep (100000);
  }
  cout << "Waiting 5 seconds without receiving packets. Quit.\n";
  return 0;
}
