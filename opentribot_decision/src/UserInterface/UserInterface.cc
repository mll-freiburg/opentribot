#include "UserInterface.h"
#include "../Structures/Journal.h"

#include "CliUserInterface/CliUserInterface.h"
#include "AddComUserInterface/AddComUserInterface.h"

Tribots::UserInterface::UserInterface(const ConfigReader& conf , WorldModel& wm) throw (TribotsException, std::bad_alloc)
{
  // Soll mit einem remote Programm kommuniziert werden?
  // das Lokale Interface wird von diesem erzeugt und hat vorrang bei der Steuerung
  bool useComUserInterface = false;
  if (conf.get("add_com_user_interface", useComUserInterface) <=0){
    JERROR("no config line \"add_com_user_interface\" found");
    throw Tribots::InvalidConfigurationException ("add_com_user_interface");
  }
  if (useComUserInterface)
    the_user_interface = new AddComUserInterface(conf,  wm);
  else
    {
      string confline;
      if (conf.get("user_interface_type", confline)<=0) {
        JERROR("no config line \"user_interface_type\" found");
      //  throw Tribots::InvalidConfigurationException ("user_interface_type");
      }
      if  (confline=="CliUserInterface")
        {
          the_user_interface = new CliUserInterface(conf,  wm);

        }
      else 
        {
          JERROR("No UserInterfaceType of this type found");
          throw Tribots::InvalidConfigurationException ("user_interface_type");
        }
    }
}

Tribots::UserInterface::~UserInterface() throw ()
{
  delete the_user_interface;
}
