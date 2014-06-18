
#ifndef _Tribots_PlayerFactory_h_
#define _Tribots_PlayerFactory_h_

#include "ImageSource.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../Structures/TribotsException.h"
#include <string>
#include <vector>
#include <map>


namespace Tribots {

  /** Abstrakte Klasse zum Erzeugen von Bildquellen; fuer jede Bildquelle
      muss eine eigene Klasse abgeleitet werden, die die Klasse bei der ImageSourceFactory
      anmeldet */
  class ImageSourceBuilder {
  public:
    /** Destruktor */
    virtual ~ImageSourceBuilder () throw () {;}
    /** erzeuge eine neue Instanz der jeweiligen Bildquelle; werfe eine Exception, wenn
        Objekt nicht ordnungsgemaess erzeugt werden kann; 
        arg1: Typbezeichnung, arg2: Config-Parameter, arg3: Section-Name fuer Config-Parameter */ 
    virtual ImageSource* get_image_source (const std::string&, const ConfigReader&, const std::string&) throw (TribotsException,std::bad_alloc) =0;
  };



  /** ImageSourceFactory verwaltet die verschiedenen Typen von Bildquellen;
      Jede Bildquelle muss sich zunaechst bei der Factory anmelden */
  class ImageSourceFactory {
  private:
    std::map<std::string, ImageSourceBuilder*> list_of_plugins; ///< die Liste bekannter Spielertypen
    static ImageSourceFactory* the_only_factory;                ///< Zeiger auf die einzige Factory (singleton)
    ImageSourceFactory () throw ();                             ///< private Konstruktor, wegen singleton-Eigenschaft
    ~ImageSourceFactory() throw ();                             ///< Destruktor auch privat, Objekt soll nicht geloescht werden

  public:
    /** statt Konstruktor: statische Aufruffunktion */
    static ImageSourceFactory* get_image_source_factory () throw (std::bad_alloc);

    /** Anmeldefunktion, arg1: Typbezeichner, arg2: Zeiger auf Builderobjekt */
    void sign_up (const std::string, ImageSourceBuilder*) throw (std::bad_alloc);
    /** Erzeugungsfunktion, arg1: Typbezeichner, arg2: Config-Parameter, arg3: Section-Name fuer Config-Parameter;
         return: neues Objekt; 
         bei Problemen werden Ausnahmen geworfen */
    ImageSource* get_image_source (const std::string&, const ConfigReader&, const std::string&) throw (TribotsException,std::bad_alloc,std::invalid_argument);
  };

}

#endif
