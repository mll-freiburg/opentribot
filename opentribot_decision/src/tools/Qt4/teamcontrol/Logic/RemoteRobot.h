
#ifndef TribotsTools_RemoteRobot_h
#define TribotsTools_RemoteRobot_h

#include "../../../../Fundamental/ConfigReader.h"
#include "../../../../Fundamental/Time.h"
#include "../../../../Structures/GameState.h"
#include "../../../../Communication/TribotsUDPCommunication.h"
#include "../States/RemoteRobotState.h"
#include <string>


namespace TribotsTools {

  /** Klasse RemoteRobot, uebernimmt Kommunikation mit Roboter */
  class RemoteRobot {
  public:
    /** Konstruktor; Arg1: Configfile fuer Einstellungen, Arg2: Roboternummer, Arg3: Robotername */
    RemoteRobot (const Tribots::ConfigReader&, unsigned int, const std::string&) throw ();
    ~RemoteRobot () throw ();

    /** Kommunikation mit den Spielern */
    void receive () throw ();
    void send () throw ();

  private:
    unsigned int robot;
    Tribots::TribotsUDPCommunication comm;
    bool field_geometry_request;
    bool playertypelist_request;
    bool playerrolelist_request;
    unsigned until_send_lines;
    Tribots::Time latest_receive;
    bool first_receive;
  };

}

#endif
