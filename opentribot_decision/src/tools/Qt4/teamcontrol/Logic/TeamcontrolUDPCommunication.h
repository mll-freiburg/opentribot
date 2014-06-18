
#ifndef _TribotsTools_TeamcontrolUDPCommunication_h_
#define _TribotsTools_TeamcontrolUDPCommunication_h_

#include "../../../../Communication/NonspecificTaggedUDPCommunication.h"
#include "../States/RemoteBlackboard.h"


namespace TribotsTools {

  /** UDP-Verbindung, abgestimmt auf die Datenstrukturen die zwischen
      mehreren teamcontrol */
  class TeamcontrolUDPCommunication : public Tribots::NonspecificTaggedUDPCommunication {
  public:
    /** Konstruktor */
    TeamcontrolUDPCommunication () throw (std::bad_alloc);
    ~TeamcontrolUDPCommunication () throw ();
    /** 'Bye' senden und Verbindung schliessen */
    void close () throw ();
    
    // Methoden zum lesen/schreiben/anfordern/Anforderung abfragen bestimmter Informationen
    // Return: true, wenn Information geschrieben werden konnte/empfangen wurde
    virtual bool putHello () throw (std::bad_alloc);
    virtual bool getHello () throw ();

    virtual bool putBye () throw (std::bad_alloc);
    virtual bool getBye () throw ();

    virtual bool putMessageID (unsigned short int) throw (std::bad_alloc);
    virtual bool getMessageID (unsigned short int&) throw ();

    virtual bool putCoachState (const CoachState&) throw (std::bad_alloc);
    virtual bool getCoachState (CoachState&, unsigned int) throw (std::bad_alloc);
    virtual unsigned int numCoachState () throw ();

    virtual bool putTeamState (const TeamState&) throw (std::bad_alloc);
    virtual bool getTeamState (TeamState&) throw (std::bad_alloc);

    virtual bool putHelpState (const HelpState&) throw (std::bad_alloc);
    virtual bool getHelpState (HelpState&) throw (std::bad_alloc);

    virtual bool putRemoteRobotState (const RemoteRobotState&) throw (std::bad_alloc);
    virtual bool getRemoteRobotState (RemoteRobotState&, unsigned int) throw (std::bad_alloc);
    virtual unsigned int numRemoteRobotState () throw ();

    virtual bool putJoystickState (const JoystickState&) throw (std::bad_alloc);
    virtual bool getJoystickState (JoystickState&) throw (std::bad_alloc);

    virtual bool putRemoteBlackboardState (const BlackboardState&) throw (std::bad_alloc);
    virtual bool getRemoteBlackboardState (BlackboardState&) throw (std::bad_alloc);
    
  };

}

#endif
