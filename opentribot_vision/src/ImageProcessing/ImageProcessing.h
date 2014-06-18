
#ifndef tribots_image_processing_h
#define tribots_image_processing_h

#include <stdexcept>
#include "ImageProcessingType.h"
#include "../Structures/TribotsException.h"
#include "../Fundamental/ConfigReader.h"
#include "../Structures/VisibleObject.h"
#include "Formation/Image.h"

namespace Tribots {

  class ImageProducer;
  class VisibleObjectList;
  class RegionList;

  /** Schnittstelle der Bildfolgenauswerte-Komponente
      Aufgaben: Anbindung der Kamera, Interpretation des Kamerabildes, Objekterkennung */
  class ImageProcessing {
  private:
    ImageProcessingType* the_image_processing;
    char* image_processing_descriptor;
    const ConfigReader& configuration_list;

    /** eigentliche Implementierung des Wechsels der Bildverarbeitung */
    void really_change_image_processing_type (const char*, const char*, const ConfigReader&, const ImageProducer*) throw (TribotsException, std::bad_alloc);
    
  public:
    /** Konstruktor
	Arg1: ConfigReader, aus dem alle relevanten Parameter ausgelesen werden, bis auf das, was in Arg2 uebergeben wird
	Arg2: Geometrie des Spielfeldes */
    ImageProcessing (const ConfigReader&, const char* type, const char* section,
                     const ImageProducer*) throw (TribotsException, std::bad_alloc);
    ~ImageProcessing () throw ();

    /** Wechsel des Bildverarbeitungstyps
	Parameter werden aus dem ConfigReader gelesen, der mit dem Konstruktor uebergeben wurde
	Arg1: Bezeichner fuer Bildverarbeitungstyp
	Ret: bei Erfolg, true */
    bool change_image_processing_type (const char*, const char*, const ImageProducer*) throw ();
    /** Wechsel des Bildverarbeitungstyps
        Arg1: Bezeichner fuer Bildverarbeitungstyp
        Arg2: Parameterliste fuer neuen Bildverarbeitungstyp
        Ret: bei Erfolg, true */
    bool change_image_processing_type (const char*, const char*, const ConfigReader&, const ImageProducer*) throw ();
    
    /** liefere eine Beschreibung der Informationsquelle und Verarbeitung */
    const char* get_image_processing_type () const throw ();

    /** liefere eine Liste aus der Bildquelle erkannter Objekte
	Seiteneffekte: - Bildquelle (Kamera/Simulator/Datei) wird ausgelesen
	               - Liste der erkannten Objekte wird an das Weltmodell uebergeben
                       - ggf. werden Informationen aus dem Weltmodell verwendet */
    inline void process_image (Image* image, VisibleObjectList* vol, RegionList* ballRegions) throw (BadHardwareException) { return the_image_processing->process_image (image, vol, ballRegions); }

    /** Request a raw image from the camera for debugging purposes. The image may be prepared during the next control cycles. */
    void request_image_raw()  throw(TribotsException) { return the_image_processing->request_image_raw (); }
    /** Request a processed image from the image processing algorithms. The visualization may be prepared during the next control cycles. */
    void request_image_processed()  throw(TribotsException) { return the_image_processing->request_image_processed(); }
    /** Ask, if the image preparation process is finished and the image is ready. */
    bool is_image_available() throw() { return the_image_processing->is_image_available(); }
    /** Collect the requested and prepared image. Do not call, before is_image_available returns true. */
    const Image* get_image()  throw(TribotsException) { return the_image_processing->get_image();}  
    /** Free the image after usage */
    void free_image() throw(TribotsException) { return the_image_processing->free_image(); }
};

}

#endif

