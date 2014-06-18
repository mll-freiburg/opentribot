
#include "RefboxClient.h"
#include <arpa/inet.h>
#include <netdb.h>
#include <iostream>
#include "../../../../Fundamental/Time.h"
#include <cstring>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;


TribotsTools::RefboxClient ::RefboxClient (const char* lname) throw () : logstream (lname) {
  latest_signal=Tribots::SIGstop;
  okayfailed=0;
  connected=false;
}

TribotsTools::RefboxClient ::~RefboxClient () throw () {
  disconnect ();
  logstream.flush();
}

namespace {
  inline int socket_connect (int socket, struct sockaddr *addr, socklen_t length) {   // Funktion wegen Namenskonfikt noetig
    return connect (socket, addr, length);
  }
}

bool TribotsTools::RefboxClient::connect (const char* ip, int port) throw () {
  Tribots::Time now;
  if (connected)
    disconnect ();     // wenn Verbinung bereits geoeffnet, erst schliessen

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  struct sockaddr_in address;
  address.sin_family = AF_INET;
  struct hostent* serveraddr;
  if ( (serveraddr = gethostbyname(ip)) == 0) {
    logstream << now << " unknown host; couldn't connect to refbox" << endl;
    return false;
  }
  memcpy(&address.sin_addr, serveraddr->h_addr, serveraddr->h_length);
  address.sin_port = htons(port);
  int result = socket_connect(sockfd, (struct sockaddr *) &address, sizeof(address));

  if (result != 0) {
    logstream << now << " couldn't connect to refbox" << endl;
    perror("Could not connect to server");
    return false;
  }

  logstream << now << " successfully connected to " << inet_ntoa(address.sin_addr) << ", " << ntohs(address.sin_port) << endl;
  connected=true;
  okayfailed=0;
  
  return true;
}

void TribotsTools::RefboxClient::disconnect() throw () {
  Tribots::Time now;
  if (connected) {
    logstream << now << " disconnect from refbox" << endl;
    close(sockfd);
    connected=false;
  }
}

RefboxClient::RefboxMessage TribotsTools::RefboxClient::listen() throw () {
  RefboxMessage res;
  res.signal = Tribots::SIGnop;
  res.messages.clear();

  Tribots::Time now;
  if (!connected)
    return res;

  // pruefen, ob gelesen werden kann
  fd_set rfds;
  struct timeval tv;
  FD_ZERO(&rfds);
  FD_SET(sockfd, &rfds);
  tv.tv_sec = 0; tv.tv_usec = 100;
  int sc = select(static_cast<int>(sockfd)+1, &rfds, NULL, NULL, &tv);
  okayfailed = ((sc==0) ? 0 : okayfailed+1);
  bool message_available = FD_ISSET(sockfd, &rfds);

  if (message_available) {
    char rcvd[10];
    for (unsigned int i=0; i<10; i++)
      rcvd[i]=0;
    int result = read(sockfd, &rcvd, 10);
//      cerr << "rcvd=" << rcvd[0] << '|' << rcvd[1] << '|' << rcvd[2] << '|' << rcvd[3] << '|' << rcvd[4] << '|' << rcvd[5] << '|' << rcvd[6] << '|' << rcvd[7] << '|' << rcvd[8] << '|' << rcvd[9] << '\n';
    if (result <0) {
      logstream << now << " error when listening for message from refbox" << endl;
      return res;     // Fehler aufgetreten
    } else if (result==0) {
      logstream << now << " server has been closed; disconnect" << endl;
      disconnect ();
    }
    if (strncmp (rcvd, "Welcome..",10)==0 || strncmp(rcvd, "Reconnect", 10)==0) {
      // get welcome message
      logstream << now << " got welcome message = '";
      for (unsigned int i=0; i<10; i++) {
        if (rcvd[i]!='\0')
          logstream << rcvd[i];
        else
          break;
      }
      logstream << "'." << endl;
      return res;
    } else {
      logstream << now << " received message from refbox: '" << rcvd[0] << "' = " << static_cast<int>(rcvd[0]) << "." << endl;
      for (unsigned int i=0; i<10; i++)
        if (rcvd[i]!=0)
          unresolved_signals.push_back (rcvd[i]);
    }
  }
  
  // jetzt umwandeln des gelesenen in Tribots::RefboxSignals
  bool relevant_signal=true;
  do {
    if (unresolved_signals.size()==0) {
      relevant_signal=false;
      break;
    }
    
    res.signal = Tribots::SIGnop;
    char rbsig = unresolved_signals.front ();
    unresolved_signals.pop_front();
    switch (rbsig) {
    case 'a': relevant_signal=false; res.signal=Tribots::SIGmagentaGoalScored; break;
    case 'A': relevant_signal=false; res.signal=Tribots::SIGcyanGoalScored; break;
    case 'H': res.signal=Tribots::SIGhalt; break;
    case 'S': res.signal=Tribots::SIGstop; break;
    case 's': 
      if (latest_signal == Tribots::SIGstop || latest_signal == Tribots::SIGhalt)
        res.signal=Tribots::SIGstart;
      else
        res.signal=Tribots::SIGready;
      break;
    case 'x': res.signal=Tribots::SIGstop; break;
    case 'k': res.signal=Tribots::SIGmagentaKickOff; break;
    case 'K': res.signal=Tribots::SIGcyanKickOff; break;
    case 'f': res.signal=Tribots::SIGmagentaFreeKick; break;
    case 'F': res.signal=Tribots::SIGcyanFreeKick; break;
    case 'g': res.signal=Tribots::SIGmagentaGoalKick; break;
    case 'G': res.signal=Tribots::SIGcyanGoalKick; break;
    case 'c': res.signal=Tribots::SIGmagentaCornerKick; break;
    case 'C': res.signal=Tribots::SIGcyanCornerKick; break;
    case 't': res.signal=Tribots::SIGmagentaThrowIn; break;
    case 'T': res.signal=Tribots::SIGcyanThrowIn; break;
    case 'p': res.signal=Tribots::SIGmagentaPenalty; break;
    case 'P': res.signal=Tribots::SIGcyanPenalty; break;
    case 'N': res.signal=Tribots::SIGdroppedBall; break;
    case '1': res.messages.push_back (string("start first half!")); relevant_signal=false; res.signal=Tribots::SIGnop; break;
    case '2': res.messages.push_back (string("start second half!")); relevant_signal=false; res.signal=Tribots::SIGnop; break;
    case 'h': res.messages.push_back (string("end first half!")); relevant_signal=false; res.signal=Tribots::SIGnop; break;
    case 'e': res.messages.push_back (string("end second half!")); relevant_signal=false; res.signal=Tribots::SIGnop; break;
    default: relevant_signal=false; res.signal=Tribots::SIGnop;
    }
  }while (!relevant_signal);
  if (relevant_signal)
    latest_signal = res.signal;
  logstream << now << " interpret refbox message as " << Tribots::refbox_signal_names[res.signal] << endl;
  return res;
}

bool TribotsTools::RefboxClient::is_okay () const throw () {
  return (okayfailed<=6) && connected;
}

bool TribotsTools::RefboxClient::is_connected () const throw () {
  return connected;
}
