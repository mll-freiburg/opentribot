
#ifndef _TribotsTools_GRABBINGTHREAD_H_
#define _TribotsTools_GRABBINGTHREAD_H_

#include "../../../../ImageProcessing/Formation/ImageSource.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/PixelAnalysis/ColorClassifier.h"
#include "../../../../ImageProcessing/ObjectAnalysis/ColorClasses.h"
#include "../../../../Fundamental/POSIXThread.h"


namespace TribotsTools {

  /** ein Thread, der staendig ein Bild von einer Bildquelle holt;
    GrabbingThread arbeitet mit drei internen Puffern, die abwechselnd beschrieben werden.
    \attention Veraenderungen des Farbklassifikationstools sind nicht sicher gegen 
    Parallelisierungsinkonsistenzen! */
  class GrabbingThread : public Tribots::POSIXThread {
  public:
    /** Argument: Bildquelle */
    GrabbingThread(Tribots::ImageSource*);
    ~GrabbingThread();

    virtual void cancel ();
    virtual void start ();
    /** anstatt neue Bilder zu holen, immer wieder das letzte Bild wiederholen */
    virtual void stopGrabbing();
    /** anstatt das letzte Bild zu wiederholen wieder neue Bilder von der Bildquelle holen */
    virtual void contGrabbing();

    /** das letzte Bild anfragen; dafuer das bisher angefragte Bild wieder freigeben
    \attention auf das erhaltene Bild nie delete ausfuehren, das Bild gehoert dem
    GrabbingThread, kann aber veraendert werden */
    virtual Tribots::Image& getImage();

    /** einen Pointer auf den Klassifizierer uebergeben, der verwendet werden soll */
    virtual void setClassifier (Tribots::ColorClassifier*);

    static Tribots::POSIXMutex mutexImageSourceAccess;
  protected:
    virtual void main ();

    Tribots::ImageSource* imageSource;
    Tribots::Image* image;   ///< das Bild, das von der Bildquelle kommt
    Tribots::Image* bufferedImage [3];   ///< das ggf. farbklassifizierte Bild, was nach aussen gegeben wird
    unsigned int client_buffer;   ///< der Puffer, der gerade vom Klienten gelesen wird
    unsigned int prepared_buffer;   ///< der Puffer, der gefuellt und vorbereitet ist, so dass Klient lesen kann (kann identisch mit client_buffer sein)
    bool goon;   ///< neue Bilder von Bildquelle holen (true) oder letztes wiederverwenden (false)
    Tribots::ColorClassifier* classifier;  ///< Pointer auf einen HSISegmentationTool
    const Tribots::ColorClassInfoList colorInfos;
  };

}


#endif
