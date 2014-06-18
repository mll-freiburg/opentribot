
#ifndef tribots_user_interface_h
#define tribots_user_interface_h


#include "../Structures/TribotsException.h"
#include "../Player/Player.h"
#include "../WorldModel/WorldModel.h"

#include "../Fundamental/ConfigReader.h"


#include "UserInterfaceType.h"

namespace Tribots {

  /** Schnittstelle der Bedienungs- und Kommunikationsschnittstelle
      Aufgaben: Anzeige relevanter Informationen, Abfrage von Benutzerinteraktionen 
      Schnittstelle dient auch zur Anbindung eines Kommunikationsmuls zur Kommunikation
      mit Zentralrechner oder zwischen verschiedenen Robotern */
  class UserInterface {
  protected:
    UserInterfaceType  * the_user_interface;

  public:
    UserInterface (const ConfigReader&, WorldModel&) throw (TribotsException, std::bad_alloc);
    ~UserInterface() throw ();

    /** Verarbeitung von Nachrichten, sammeln von Informationen, Benutzinteraktion
        Rueckgabewert: false, wenn Programm beendet werden soll, true sonst */
    inline bool process_messages () throw () {return the_user_interface->process_messages();};
  };
  
}

#endif

