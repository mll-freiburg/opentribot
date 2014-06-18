
#ifndef _TribotsTools_TeamcontrolSlaveClient_h_
#define _TribotsTools_TeamcontrolSlaveClient_h_

#include "TeamcontrolUDPCommunication.h"

namespace TribotsTools {

  /** Client auf Slave-Seite fuer die Kommunikation zwischen mehreren Teamcontrols */
  class TeamcontrolSlaveClient {
  private:
    TeamcontrolUDPCommunication comm;
    std::vector<unsigned int> robot_ids_received;
    BlackboardState latest_own_state;
    Tribots::Time latest_receive_time;
    unsigned short int message_id;
    unsigned int wrong_mid;
  public:
    TeamcontrolSlaveClient () throw ();
    ~TeamcontrolSlaveClient () throw ();

    /** Mit einem Masterteamcontrol verbinden, arg1=IP-Adresse, arg2=port */
    bool connect (const std::string&, unsigned int) throw ();
    /** Verbindung aufloesen */
    void unconnect () throw ();
    /** true, wenn zuor 'connect' aufgerufen wurde */
    bool is_connected () const throw ();

    /** Blackboard empfangen und Information einarbeiten, liefert true bei Erfolg */
    bool receive () throw ();
    /** Blackboard versenden */
    bool send () throw ();

    /** Info ueber Kommunikation */
    std::string commStatus () const throw ();
    /** Info ueber Kommunikation */
    bool comm_interrupted () const throw ();
  };

}


#endif
