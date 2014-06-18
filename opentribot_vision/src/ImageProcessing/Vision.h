
#ifndef tribots_vision_h
#define tribots_vision_h

#include <stdexcept>
#include "../Structures/TribotsException.h"
#include "../Fundamental/ConfigReader.h"
#include "VisionType.h"
#include "ObjectAnalysis/Regions.h"

namespace Tribots {

  class VisibleObjectList;
  class Image;

  /** Schnittstelle der Vision-Komponente
      Aufgaben: Anbindung der Kameras, Interpretation der Kamerabilder, Objekterkennung */
  class Vision {
  private:
    VisionType* the_vision;
    char* vision_descriptor;
    const ConfigReader& configuration_list;

    /** eigentliche Implementierung des Wechsels des Vision-subystems */
    void really_change_vision_type (const char*, const char*, const ConfigReader&) throw (TribotsException, std::bad_alloc);
    
  public:
    /** Konstruktor
	Arg1: ConfigReader, aus dem alle relevanten Parameter ausgelesen werden, bis auf das, was in Arg2 uebergeben wird
	Arg2: Geometrie des Spielfeldes */
    Vision (const ConfigReader&) throw (TribotsException, std::bad_alloc);
    ~Vision () throw ();

    /** Wechsel des Bildverarbeitungstyps
	Parameter werden aus dem ConfigReader gelesen, der mit dem Konstruktor uebergeben wurde
	Arg1: Bezeichner fuer Bildverarbeitungstyp
	Ret: bei Erfolg, true */
    bool change_vision_type (const char*, const char*) throw ();
    /** Wechsel des Bildverarbeitungstyps
        Arg1: Bezeichner fuer Bildverarbeitungstyp
        Arg2: Parameterliste fuer neuen Bildverarbeitungstyp
        Ret: bei Erfolg, true */
    bool change_vision_type (const char*, const char*, const ConfigReader&) throw ();
    
    /** liefere eine Beschreibung der Informationsquelle und Verarbeitung */
    const char* get_vision_type () const throw ();
    
    /** liefere die Anzahl der angeschlossenen Bildquellen */
    int get_num_sources() const throw() { return the_vision->get_num_sources(); }

    /** liefere eine Liste von Listen aus den einzelnen Bildquellen erkannter Objekte
	Seiteneffekte: - Bildquellen (Kamera/Simulator/Datei) werden ausgelesen
	               - Liste der erkannten Objekte wird an das Weltmodell uebergeben
                       - ggf. werden Informationen aus dem Weltmodell verwendet */
    inline void process_images () throw (BadHardwareException) { return the_vision->process_images (); }

    inline const VisibleObjectList& get_last_seen_objects(int camera) const throw()
    { return the_vision->get_last_seen_objects(camera); }  // FIXME: auch alle Kameras?!
    
    /** Debug: return the list of regionlists (one for each camera) */
    const std::vector<RegionList*>& get_regionlists() throw(TribotsException) { return the_vision->get_regionlists(); }
   
    /** Request a raw image from the camera for debugging purposes. The image may be prepared during the next control cycles. */
    void request_image_raw(int camera)  throw(TribotsException) { return the_vision->request_image_raw (camera); }
    /** Request a processed image from the image processing algorithms. The visualization may be prepared during the next control cycles. */
    void request_image_processed(int camera)  throw(TribotsException) { return the_vision->request_image_processed(camera); }
    /** Ask, if the image preparation process is finished and the image is ready. */
    bool is_image_available(int camera) throw() { return the_vision->is_image_available(camera); }
    /** Collect the requested and prepared image. Do not call, before is_image_available returns true. */
    const Image* get_image(int camera)  throw(TribotsException) { return the_vision->get_image(camera);}  
    /** Free the image after usage */
    void free_image(int camera) throw(TribotsException) { return the_vision->free_image(camera); }
};

}

#endif

