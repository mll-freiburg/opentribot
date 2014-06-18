#ifndef tribots_vision_type_h
#define tribots_vision_type_h

#include <vector>

#include "../Structures/TribotsException.h"
#include "../Structures/VisibleObject.h"
#include "ObjectAnalysis/Regions.h"


namespace Tribots {

  class Image;

  /** abstrakte Klasse zur Modellierung einer Bildfolgenauswertung
      Schnittstelle der Bildfolgenauswertungs-Komponente nach innen */
  class VisionType {
  public:
    virtual ~VisionType () throw () {;}
    
    virtual int get_num_sources() const throw() =0;

    /** liefere eine Liste aus der Bildquelle erkannter Objekte
	Seiteneffekte: - Bildquelle (Kamera/Simulator/Datei) wird ausgelesen
	               - Liste der erkannten Objekte wird an das Weltmodell uebergeben
                       - ggf. werden Informationen aus dem Weltmodell verwendet */
    virtual void process_images () throw (BadHardwareException) =0;
    
    // Fuer debug-Zwecke:
    virtual const VisibleObjectList& get_last_seen_objects(int camera) const throw()
    { return lastSeenObjects.at(camera); }

    virtual const std::vector<RegionList*>& get_regionlists() throw(TribotsException) { throw TribotsException("regionlists not available. This feature is not implemented in this ImageProcessingType."); }

    virtual void request_image_raw(int) throw(TribotsException) {;}
    virtual void request_image_processed(int) throw(TribotsException) {;}
    virtual bool is_image_available(int) throw() { return false; }
    virtual const Image* get_image(int) throw(TribotsException) { throw TribotsException("No image ready. This feature is not implemented in this ImageProcessingType."); }
    virtual void free_image(int) throw(TribotsException) {;}

  protected:
    std::vector<VisibleObjectList> lastSeenObjects;
  };

}

#endif

