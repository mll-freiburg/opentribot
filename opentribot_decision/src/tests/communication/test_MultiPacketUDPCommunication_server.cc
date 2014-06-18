
#include <iostream>
#include <cstdlib>
#include "../../Communication/MultiPacketUDPCommunication.h"

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc<2) {
    cerr << argv[0] << " Port\n";
    return -1;
  }
  int port = atoi (argv[1]);
  MultiPacketUDPCommunication sock;
  sock.init_as_server (port);

  vector<char*> buffers;
  vector<unsigned int> buflens;

  while (true) {
    unsigned int success = sock.receive (buffers, buflens, 8192);
    if (buffers.size()>0) {
      cout << "Received " << buffers.size() << " Buffers ";
      for (unsigned int i=0; i<buffers.size(); i++) {
	cout << '[';
	for (unsigned int j=0; j<buflens[i]; j++)
	  cout << buffers[i][j];
	cout << "] ";
	delete [] buffers[i];
      }
      cout << '\n';
    }
    cout << "TransferSuccess: " << success;
    std::cout << "; Started=" << sock.started() << '\n';
    sleep (3);
  }

  sock.close();
  return 0;
}
