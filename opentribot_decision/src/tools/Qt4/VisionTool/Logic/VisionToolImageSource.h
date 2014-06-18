
#ifndef _TribotsTools_VisionToolImageSource_h_
#define _TribotsTools_VisionToolImageSource_h_

#include "../../../../ImageProcessing/Formation/Image.h"
#include "../../../../ImageProcessing/Formation/ImageSource.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/ObjectAnalysis/ColorClasses.h"
#include "../../../../Fundamental/ConfigReader.h"
#include "../../../../ImageProcessing/Formation/IIDC.h"
#include "GrabbingThread.h"
#include "ColorTools.h"


namespace TribotsTools {

  class VisionToolImageSource {
  public:
    VisionToolImageSource (Tribots::ConfigReader& cfg) throw ();
    ~VisionToolImageSource () throw ();

    /** Modus setzen:
        singlePicture: Bilder werden nur geholt und verarbeitet, wenn mittels getImage angefragt wird,
        not singlePicture: Bilder werden im Hintergrund kontinuierlich geholt und verarbeitet und koennen mit getImage abgeholt werden
        classify: Bilder bereits farbklassifizieren
        not classify: Originalbilder liefern */
    void setMode (bool singlePicture =true, bool classify =false);

    /** Bildquelle davon informieren, dass sich evtl. Einstellungen im ConfigReader zum Attribut key geaendert haben */
    void notify (const std::string& key);
    void notify (const char* key) { notify (std::string(key)); }

    /** Bild abholen. Bild gehoert dem VisionToolImageSource, kann aber veraendert werden */
    Tribots::Image& getImage ();

    /** wurde die Bildquelle bereits gestartet? */
    bool isStarted () const throw ();

    /** Bilder aus einer Datei lesen; Parameteruebergabe implizit ueber ConfigReader */
    void startFileSource () throw (Tribots::TribotsException);

    /** Kamera starten; Parameteruebergabe implizit ueber ConfigReader */
    void startCameraSource () throw (Tribots::TribotsException);

    /** Bildquelle (Datei, Kamera) freigeben */
    void stop () throw ();

    /** Kamera (oder NULL-Pointer) zurueckbekommen */
    Tribots::IIDC* getCamera () throw ();

    /** Eine Liste mit verfuegbaren Kamera-UIDs liefern */
    std::vector<std::string> getUIDs () throw (Tribots::TribotsException);

    /** Den Classifier liefern */
    Tribots::HSISegmentationTool& getClassifier () throw ();
  private:
    Tribots::ConfigReader& config;
    Tribots::ImageSource* imageSource;
    Tribots::RGBImage defaultImage;
    Tribots::Image* returnImage;
    bool isCamera;
    std::string section;
    GrabbingThread* grabThread;
    bool useGrabbingThread;
    bool useColorClassifier;
    Tribots::HSISegmentationTool classifier;
    const Tribots::ColorClassInfoList colorInfos;
  };

}

#endif
