
#ifndef _Tribots_ImageProcessingFactory_h_
#define _Tribots_ImageProcessingFactory_h_

#include "ImageProcessingType.h"
#include "../Structures/TribotsException.h"
#include "../Fundamental/ConfigReader.h"
#include <string>
#include <map>


namespace Tribots {
  class ImageProducer;

  /** Abstrakte Klasse zum Erzeugen von Bildverarbeitungstypen; fuer jeden Bildverarbeitungtyp
      muss eine eigene Klasse abgeleitet werden, die die Klasse bei der ImageProcessingFactory
      anmeldet. Als Beispiel siehe die ImageProcessingDummy-Klasse */
  class ImageProcessingBuilder {
  public:
    /** Destruktor */
    virtual ~ImageProcessingBuilder () throw () {;}
    /** erzeuge eine neue Instanz des jeweiligen Bildverarbeitungs; werfe eine Exception, wenn
	Objekt nicht ordnungsgemaess erzeugt werden kann; 
	arg1: Typbezeichnung, arg2: Config-Parameter, arg3: NULL-Pointer oder weiteres Bildverarbeitung, 
	falls Bildverarbeitungen verschachtelt werden sollen */ 
    virtual ImageProcessingType* get_image_processing (const std::string&, const std::string&, const ConfigReader&, ImageProcessingType*, const ImageProducer*) throw (TribotsException,std::bad_alloc) =0;
  };



  /** ImageProcessingFactory verwaltet die verschiedenen Typen von Bildverarbeitungstypen;
      Jeder Bildverarbeitungstyp muss sich zunaechst bei der Factory anmelden */
  class ImageProcessingFactory {
  private:
    std::map<std::string, ImageProcessingBuilder*> list_of_plugins; ///< die Liste bekannter Bildverarbeitungstypen
    static ImageProcessingFactory* the_only_factory;                ///< Zeiger auf die einzige Factory (singleton)
    ImageProcessingFactory () throw ();                             ///< private Konstruktor, wegen singleton-Eigenschaft
    ~ImageProcessingFactory() throw ();                             ///< Destruktor auch privat, Objekt soll nicht geloescht werden

  public:
    /** statt Konstruktor: statische Aufruffunktion */
    static ImageProcessingFactory* get_image_processing_factory () throw (std::bad_alloc);

    /** Anmeldefunktion, arg1: Typbezeichner, arg2: Zeiger auf Builderobjekt */
    void sign_up (const std::string, ImageProcessingBuilder*) throw (std::bad_alloc);
    /** Erzeugungsfunktion, arg1: Typbezeichner, arg2: Section in der Config-Datei, arg3: Config-Parameter, return: neues Objekt; 
	bei Problemen werden Ausnahmen geworfen */
    ImageProcessingType* get_image_processing (const std::string, const std::string section, const ConfigReader&, const ImageProducer*) throw (TribotsException,std::bad_alloc,std::invalid_argument);
  };

}

#endif
