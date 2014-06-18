
#ifndef _TribotsTools_PingClient_h_
#define _TribotsTools_PingClient_h_

#include <unistd.h>
#include <string>
#include "../../../Fundamental/POSIXThread.h"

namespace TribotsTools {

  /** Thread, der parallel einen Ping laufen laesst und die Pingzeiten
      exponentiell glaettet. Ist die pingzeit negativ, ist die Verbindung
      gestoert oder der gepingte Rechner existiert nicht */
  class PingClient : public Tribots::POSIXThread {
  public:
    PingClient ();
    ~PingClient ();
    /** die Adresse des zu pingenden Rechners setzen */
    void setAddress (const std::string&);
    /** die Adresse, auf die gerade gepingt wird */
    std::string getAddress ();
    void exit ();
    /** die durchschnittliche Antwortzeit in ms (exponentiell geglaettet).
        Negative Werte bedeuten gestoerte Verbindung */
    double getPingTime () throw ();

  private:
    void threadCanceled ();
    void main ();
    double responseTime (const std::string&);

    std::string inetaddress;
    bool addressChanged;
    pid_t childPID;
    int pipeDescriptor [2];
    double latestPingTime;
    Tribots::POSIXMutex mutexAccessAddress;
    Tribots::POSIXMutex mutexAccessPingTime;
  };

}

#endif
