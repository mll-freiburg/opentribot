
#include <iostream>
#include <sstream>
#include <cstdlib>
#include "../../Communication/UDPSocket.h"

using namespace std;
using namespace Tribots;

int main (int argc, char** argv) {
  if (argc<3) {
    cerr << argv[0] << " host port\n";
    return -1;
  }
  int port = atoi (argv[2]);
  UDPSocket sock;
  sock.init_as_client (argv[1], port);

  char buffer [201];

  unsigned long int li=0;
  while (true) {
    unsigned int len=0;
    /*
    cout << "Input: ";
    char c;
    while (cin.get(c) && c != '\n') {
      buffer[len]=c;
      len++;
    }
    buffer[len++]='\0';
    */
    ostringstream os;
    os << "a" << li;
    string s = os.str();
    len = s.length()+1;
    for (unsigned int i=0; i+1<len; i++)
      buffer[i]=s[i];
    buffer[len-1]='\0';
    
    sock.send (buffer, len);
    std::cerr << "sending message [" << buffer << "]\n";
    std::cerr << "Started=" << sock.started() << "; Okay=" << sock.okay() << '\n';
    
    if (sock.receive (buffer, len, 200)) {
      buffer[len]='\0';
      std::cerr << "receiving message [" << buffer << "]\n";
    }

    usleep (400000);
    li++;
  }
  
  sock.close();
  return 0;
}
