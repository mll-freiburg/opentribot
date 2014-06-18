
#ifndef tribots_add_com_user_interface_h
#define tribots_add_com_user_interface_h

#include "../../Structures/TribotsException.h"
#include "../../Fundamental/Time.h"
#include "../UserInterfaceType.h"
#include "../../Communication/TribotsUDPCommunication.h"


#include "../../WorldModel/WorldModel.h"

#include "../../Structures/TacticsBoardDescription.h"
#include <fstream>
#include <string>

namespace Tribots {

  class Player;
  class Vision;

  /** Schnittstelle zur Kommunikation mit Teamcontrol mit
      Seiteneffekte: greift auf das Weltmodell und den Spieler zu (lesend und schreibend) */
  class AddComUserInterface : public UserInterfaceType {
  public:
    AddComUserInterface (const ConfigReader&,  WorldModel&) throw (Tribots::TribotsException, std::bad_alloc);
    ~AddComUserInterface () throw ();
    /** Kommunikation bearbeiten */
    bool process_messages () throw ();


  private:
    

    UserInterfaceType* the_local_user_interface;
    WorldModel& the_world_model;
    TribotsUDPCommunication comm;
    
    Time latest_message_board;
    Time latest_message_board_request;
    int max_message_board_fail;
    bool tournament_mode;
    Time timestamp_latest_comm;
    std::ofstream* synch_out;
    
    bool requestedImage;             ///< wurde ein Bild angefragt?
    std::string debug_image_filename_base;

    std::vector<TacticsAttribute> force_tactics;  ///< Taktik-Parameter, zu denen das Teamcontrol gezwungen werden soll
  };
  
}

#endif
