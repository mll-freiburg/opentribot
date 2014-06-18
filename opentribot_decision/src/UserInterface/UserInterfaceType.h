#ifndef tribots_user_interface_type_h
#define tribots_user_interface_type_h


namespace Tribots {

  /** Schnittstelle der Bedienungs- und Kommunikationsschnittstelle
      Aufgaben: Anzeige relevanter Informationen, Abfrage von Benutzerinteraktionen 
      Schnittstelle dient auch zur Anbindung eines Kommunikationsmuls zur Kommunikation
      mit Zentralrechner oder zwischen verschiedenen Robotern */
  class UserInterfaceType {
  public:
    virtual ~UserInterfaceType() throw () {;}

    /** Verarbeitung von Nachrichten, sammeln von Informationen, Benutzinteraktion
	Rueckgabewert: false, wenn Programm beendet werden soll, true sonst */
    virtual bool process_messages () throw () =0;
  };
  
}

#endif
