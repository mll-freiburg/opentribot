
#include <iostream>
#include <cstdlib>
#include <sstream>
#include "../../Communication/MultiPacketUDPCommunication.h"

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc<3) {
    cerr << argv[0] << " Host Port\n";
    return -1;
  }
  int port = atoi (argv[2]);
  MultiPacketUDPCommunication sock;
  sock.init_as_client (argv[1], port);

  vector<char*> buffers (3);
  for (unsigned int j=0; j<3; j++) {
    char* pt = new char [200];
    buffers[j] = pt;
  }
  vector<unsigned int> buflens (3);

  unsigned long int li=0;
  while (true) {
    ostringstream os;
    os << li;
    string s = os.str();
    int len = s.length()+1;
    for (unsigned int j=0; j<3; j++) {
      buffers[j][0]='a'+j;
      for (int i=0; i+1<len; i++)
	buffers[j][i+1]=s[i];
      buffers[j][len]='\0';
      buflens[j]=len+1;
    }
    
    unsigned int success = sock.send (buffers, buflens);
    std::cerr << "sending " << buffers.size() << " messages " << success << "\n";
    std::cerr << "Started=" << sock.started() << '\n';
    
    usleep (400000);
    li++;
  }
  
  sock.close();
  return 0;
}
