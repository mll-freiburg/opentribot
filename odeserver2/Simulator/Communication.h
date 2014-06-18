
#ifndef _TribotsSim_Communication_h_
#define _TribotsSim_Communication_h_

#include "../Communication/SimulatorUDPCommunication.h"
#include "../Fundamental/Time.h"
#include <vector>
#include "World.h"

namespace TribotsSim {

  /** Komminaktionsklasse auf Serverseite */
  class Communication {
  public:
    Communication ();
    ~Communication ();

    void communicate (World&);
    
  private:
    SimulatorUDPCommunication comm;
    std::vector<bool> partner_active;
    std::vector<struct sockaddr_in> partner_addresses;
    std::vector<Tribots::Time> partner_timeout;
  };

}

#endif
