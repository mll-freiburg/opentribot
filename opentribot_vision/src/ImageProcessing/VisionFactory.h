
#ifndef _Tribots_VisionFactory_h_
#define _Tribots_VisionFactory_h_

#include "VisionType.h"
#include "../Structures/TribotsException.h"
#include "../Fundamental/ConfigReader.h"
#include <string>
#include <map>


namespace Tribots {

  /** Abstrakte Klasse zum Erzeugen von Bildverarbeitungstypen; fuer jeden Bildverarbeitungtyp
      muss eine eigene Klasse abgeleitet werden, die die Klasse bei der ImageProcessingFactory
      anmeldet. Als Beispiel siehe die ImageProcessingDummy-Klasse */
  class VisionBuilder {
  public:
    /** Destruktor */
    virtual ~VisionBuilder () throw () {;}
    /** erzeuge eine neue Instanz des jeweiligen Bildverarbeitungs; werfe eine Exception, wenn
	Objekt nicht ordnungsgemaess erzeugt werden kann; 
	arg1: Typbezeichnung, arg2: Einzulesende Sektion der Konfigurationsdatei, arg3: Config-Parameter, arg4: NULL-Pointer oder weiteres Bildverarbeitung, 
	falls Bildverarbeitungen verschachtelt werden sollen */ 
    virtual VisionType* get_vision (const std::string&, const std::string&, const ConfigReader&, VisionType*) throw (TribotsException,std::bad_alloc) =0;
  };



  /** ImageProcessingFactory verwaltet die verschiedenen Typen von Bildverarbeitungstypen;
      Jeder Bildverarbeitungstyp muss sich zunaechst bei der Factory anmelden */
  class VisionFactory {
  private:
    std::map<std::string, VisionBuilder*> list_of_plugins; ///< die Liste bekannter Bildverarbeitungstypen
    static VisionFactory* the_only_factory;                ///< Zeiger auf die einzige Factory (singleton)
    VisionFactory () throw ();                             ///< private Konstruktor, wegen singleton-Eigenschaft
    ~VisionFactory() throw ();                    ///< Destruktor auch privat, Objekt soll nicht geloescht werden

  public:
    /** statt Konstruktor: statische Aufruffunktion */
    static VisionFactory* get_vision_factory () throw (std::bad_alloc);

    /** Anmeldefunktion, arg1: Typbezeichner, arg2: Zeiger auf Builderobjekt */
    void sign_up (const std::string, VisionBuilder*) throw (std::bad_alloc);
    /** Erzeugungsfunktion, arg1: Typbezeichner, arg2: Einzulesende Sektion der Konfigurationsdatei, arg3: Config-Parameter, return: neues Objekt; 
	bei Problemen werden Ausnahmen geworfen */
    VisionType* get_vision (const std::string, const std::string, const ConfigReader&) throw (TribotsException,std::bad_alloc,std::invalid_argument);
  };

}

#endif
