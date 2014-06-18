#ifndef _imageproducer_h_
#define _imageproducer_h_

#include "Image.h"
#include "ImageMonitor.h"
#include "ImageSource.h"
#include "../PixelAnalysis/CoordinateMapping.h"
#include "../PixelAnalysis/RobotMask.h"
#include "../../Structures/TribotsException.h"
#include "../../Fundamental/ConfigReader.h"
#include <string>

namespace Tribots {
  
  /**
   * Die Klasse ImageProducer ist eine Fassade für die verschiedenen
   * Bildquellen und Bildumwandlungsmethoden. Der ImageProducer kennt alle
   * zur Verfügung stehenden Implementierungen (sowohl alle von ImageSource
   * abgeleiteten Klassen, als auch alle verfügbaren Bildklassen, die von
   * Image abgeleitet wurden) und weiß wie sie zu erzeugen und zu konfigurieren
   * sind. Die zu verwendenden Implementierungen werden in einer 
   * Konfigurationsdatei ausgewählt und eingestellt.
   *
   * Ein Benutzer sollte lediglich diese Klasse und das Interface Image
   * kennen müssen (und natürlich die möglichen Einstellungen in der
   * Konfigurationsdatei ;).
   *
   * Um mehrere Image-quellen gleichzeitig benutzen zu können, kann dem
   * Konstruktor optional ein postfix string übergeben werden, mit dem
   * der konkrete Bereich der Configdateien für den jeweiligen Producer
   * festgelegt werden kann.
   */
  class ImageProducer {
  public:
    /**
     * Der Konstruktor liest die Konfiguration vom übergebenen ConfigReader.
     * An Hand der Konfiguration wird entschieden, welche Bildquelle 
     * instantiert und welches Bildformat von der Bildklasse verwendet wird.
     *
     * \param config ConfigReader, von dem die Parameter gelesen werden sollen
     * \param force_blocking wenn true, wird unabhaengig von Konfigurationseintraegen blockierend gelesen
     */
    ImageProducer(const ConfigReader& config, bool force_blocking=false) 
      throw(TribotsException);

    /** Konstruktor, der die Angabe des
     *  image_handlers (ImageProducer settings) im image_producer.cfg config
     *  file erlaubt. Es werden die sections mit den entsprechenden Namen
     *  ausgewertet.
     */
    ImageProducer(const ConfigReader& config, const std::string& image_handler, bool force_blocking=false)
      throw(TribotsException);


    /**
     * Der Destruktor schließt die Bildquelle wieder. 
     *
     * \attention Bilder, die von einem ImageProducer geliefert wurden,
     *            sind nach der Zerstörrung dieses ImageProducers unter 
     *            Umständen ungültig.
     */
    ~ImageProducer();

    /**
     * Liefert das nächste Bild von der Bildquelle. Tritt beim Holen des
     * Bildes ein Hardwarefehler auf, wird die Operation einige Male 
     * wiederholt, bevor ggf. ein schwerer (irreperabler) Hardwarefehler
     * ausgelöst wird.
     *
     * \returns das nächste Bild
     */
    Image* nextImage() throw (ImageFailed, BadHardwareException);

    enum { TYPE_RGBImage=0, TYPE_UYVYImage, TYPE_YUVImage };

    /** einen Pointer auf die Bildquelle anfragen.
     *  sollte mit Vorsicht genossen werden.
     *  nur fuer Tools und zu Debug-Zwecken vorgesehen.
     */
    ImageSource* getImageSource () throw ();

    /** liefere den zur Bildquelle passenden Bildmittelpunkt; falls unbekannt, werden negative Werte geliefert */
    Vec getImageCenter () const throw ();
    /** liefere die zur Bildquelle passende Bild->Welt-Abbildung; falls nicht vorhanden, liefere NULL */
    const ImageWorldMapping* getImageWorldMapping () const throw ();
    /** liefere die zur Bildquelle passende Welt->Bild-Abbildung; falls nicht vorhanden, liefere NULL */
    const WorldImageMapping* getWorldImageMapping () const throw ();
    /** liefere die Robotermaske zur Bildquelle; falls nicht vorhanden, liefere NULL */
    const RobotMask* getRobotMask () const throw ();
    /** ersetzen der Robotermaske durch arg1 */
    void setRobotMask (const RobotMask&) throw ();
    
    int getWidth() const throw() { return imgSource->getWidth(); }
    int getHeight() const throw() { return imgSource->getHeight(); }
    
    /** returns the origin of the camera in the robot coordinate system */
    Vec3D getCameraOrigin() const throw() { return cameraOrigin; }
        
  protected:
    /** Used by both Constructors to do the actual initialization of the
     *  ImageProducer.
     */
    void init(const ConfigReader& config, const std::string& image_handler, bool force_blocking);
    /** Initialize ImageHandler */
    void initHandler (std::string& imageSourceSectionName, std::string& sourceType, const std::string& imageHandlerSectionName, const ConfigReader& config);
    /** Initialize ColorClassifier */
    void initColorClassifier (const std::string& imageSourceSectionName, const ConfigReader& config);
    /** Initialize the ImageMonitor */
    void initMonitor (const std::string& imageHandlerSectionName, const ConfigReader& config);
    /** Initialize imageCenter and Mappings */
    void initMappings (const std::string& imageSourceSectionName, const ConfigReader& config);
    /** Initialize robot mask */
    void initRobotMask (const std::string& imageSourceSectionName, const ConfigReader& config);

    ImageSource* imgSource;  ///< Zeiger auf die aktive Bildquelle
    ColorClassifier* classifier;  ///< Zeiger auf den verwendeten Klassifikator
    ImageMonitor* monitor;  ///< Zeiger auf den zu verwendenden Monitor
    int imageType;         ///< zu verwendender Bildtyp (TYPE_RGBImage,...)

    Vec imageCenter;
    ImageWorldMapping* image2world;
    WorldImageMapping* world2image;
    RobotMask* robotMask;
    
    int camera_delay;      ///< Zeitverzoegerung der Kamera in ms

    ImageBuffer buffer;     ///< Used to buffer the images in nextImage(); ???
    Vec3D cameraOrigin;
        
  };
}


#endif
