
#ifndef _TribotsTools_TeamcontrolMasterServer_h_
#define _TribotsTools_TeamcontrolMasterServer_h_

#include "TeamcontrolUDPCommunication.h"
#include <vector>
#include <deque>

namespace TribotsTools {

  /** Server auf Master-Seite fuer die Kommunikation zwischen mehreren Teamcontrols */
  class TeamcontrolMasterServer {
  private:
    struct PartnerStruct {
      std::deque<BlackboardState> latest_own_state;
      std::deque<unsigned short int> latest_own_state_msg_id;
      BlackboardState latest_other_state;
      bool latest_other_state_valid;
      bool do_send;
      Tribots::Time latest_receive_time;
      struct sockaddr_in address;
      unsigned short int message_id;
    };
    
    TeamcontrolUDPCommunication comm;
    std::vector<PartnerStruct> clients;
  public:
    TeamcontrolMasterServer () throw ();
    ~TeamcontrolMasterServer () throw ();

    /** Einen Server-Socket auf port (arg1) anlegen */
    bool connect (unsigned int) throw ();
    /** Verbindung aufloesen */
    void unconnect () throw ();
    /** true, wenn zuor 'connect' aufgerufen wurde */
    bool is_connected () const throw ();

    /** Blackboard empfangen und Information einarbeiten, liefert true bei Erfolg */
    bool receive () throw ();
    /** Blackboard versenden */
    bool send () throw ();

    /** Info ueber Clients */
    std::string commStatus () const throw ();
  };

}


#endif
