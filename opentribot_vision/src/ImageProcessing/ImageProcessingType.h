
#ifndef tribots_image_processing_type_h
#define tribots_image_processing_type_h

#include "../Structures/TribotsException.h"
#include "../Structures/VisibleObject.h"
#include "ObjectAnalysis/Regions.h"
#include "Formation/Image.h"
#include "ObjectAnalysis/Regions.h"

namespace Tribots {

  /** abstrakte Klasse zur Modellierung einer Bildfolgenauswertung
      Schnittstelle der Bildfolgenauswertungs-Komponente nach innen */
  class ImageProcessingType {
  public:
    virtual ~ImageProcessingType () throw () {;}

    /** liefere eine Liste aus der Bildquelle erkannter Objekte
	Seiteneffekte: - Bildquelle (Kamera/Simulator/Datei) wird ausgelesen
	               - Liste der erkannten Objekte wird an das Weltmodell uebergeben
                       - ggf. werden Informationen aus dem Weltmodell verwendet */
    virtual void process_image (Image*, VisibleObjectList*, RegionList*) throw (BadHardwareException) =0;
    
    virtual void request_image_raw() throw(TribotsException) {;}
    virtual void request_image_processed() throw(TribotsException) {;}
    virtual bool is_image_available() throw() { return false; }
    virtual const Image* get_image() throw(TribotsException) { throw TribotsException("No image ready. This feature is not implemented in this ImageProcessingType."); }
    virtual void free_image() throw(TribotsException) {;}
  };

}

#endif

