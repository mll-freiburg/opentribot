#ifndef _multiimageproducer_h_
#define _multiimageproducer_h_

#include <vector>
#include "ImageProducer.h"

namespace Tribots {

  class ImageSource;

  /**
   * MultiImageProducer ist ein Container, der mehrere ImageProducer enthält -
   * also eine Fassade zum erzeugen neuer Bilder unabhängig von deren Quelle
   * und Format.
   * Wird nur eine Bildquelle benutzt, so ist MultiImageProducer zu
   * ImageProducer equivalent.
   *
   */
  class MultiImageProducer {
  public:
    /**
     * Initializes the MultiImageProducer and creates all its ImageProducers.
     * Reads from the config the number of ImageProducers that should be constructed
     * and initializes them by calling their constructor with a suffix string which
     * specifies the parts of the config that should be used.
     *
     * \param config ConfigReader, von dem die Parameter gelesen werden sollen
     */
    MultiImageProducer(const ConfigReader& config);

    /**
     * Der Destruktor schließt alle Bildquellen wieder.
     *
     * \attention Bilder, die von einem MultiImageProducer geliefert wurden,
     *            sind nach der Zerstörrung dieses MultiImageProducers unter
     *            Umständen ungültig.
     */
    ~MultiImageProducer();

    /**
     * Liefert das nächste Bild von der ersten Bildquelle. Tritt beim Holen des
     * Bildes ein Hardwarefehler auf, wird die Operation einige Male
     * wiederholt, bevor ggf. ein schwerer (irreperabler) Hardwarefehler
     * ausgelöst wird.
     *
     * \returns das nächste Bild
     */
    Image* nextImage() throw (ImageFailed, BadHardwareException);

    /**
     * Liefert das nächste Bild von der durch producer_id spezifizierten
     * Bildquelle. Die producer_id ist durch die Reihenfolge des erstellens
     * bestimmt: die erste Bildquelle hat id = 0 usw.
     * Tritt beim Holen des Bildes ein Hardwarefehler auf, wird die Operation
     * einige Male wiederholt, bevor ggf. ein schwerer (irreperabler) Hardware-
     * fehler ausgelöst wird.
     *
     * \returns das nächste Bild von Imageproducer #producer_id
     */
    Image* nextImage(int producer_id) throw (ImageFailed, BadHardwareException);

    /**
     * Returns the number of ImageProducers.
     */
    int numSources(){ return producers.size(); }

    /** liefere den zur Bildquelle passenden Bildmittelpunkt; falls unbekannt, werden negative Werte geliefert */
    Vec getImageCenter (unsigned int producer_id = 0) const throw ();
    /** liefere die zur Bildquelle passende Bild->Welt-Abbildung; falls nicht vorhanden, liefere NULL */
    const ImageWorldMapping* getImageWorldMapping (unsigned int producer_id = 0) const throw ();
    /** liefere die zur Bildquelle passende Welt->Bild-Abbildung; falls nicht vorhanden, liefere NULL */
    const WorldImageMapping* getWorldImageMapping (unsigned int producer_id = 0) const throw ();
    /** liefere die Robotermaske zur Bildquelle; falls nicht vorhanden, liefere NULL */
    const RobotMask* getRobotMask (unsigned int producer_id = 0) const throw ();

  protected:
    std::vector <ImageProducer*> producers;///< Zeiger auf die ImageProducer

  };
}

#endif
