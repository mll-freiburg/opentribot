#ifndef _Tribots_PPMIO_h_
#define _Tribots_PPMIO_h_

#include "ImageIO.h"

namespace Tribots {

  
  /**
 * Implementierung des ImageIO - Interfaces, die Bilder im PPM Format lesen
 * und schreiben kann.
   */
  class PPMIO : public ImageIO {
    public:
      virtual void write(const ImageBuffer& image, std::ostream& out) const
          throw(TribotsException);
    /**
       * PPMIO::read liest RGB Bilder aus PPM Dateien. Wird als erstes Argument
       * kein Nullzeiger übergeben, muss das übergebene Image vom Typ RGBImage
       * sein und die richtige Größe haben. Wird ein Nullzeiger übergeben,
       * legt read ein neues RGBImage passender Größe an, dass in den Bezitz des
       * Aufrufers übergeht.
       * 
       * \param *image RGBImage mit passender Größe oder Nullzeiger
       * \param filename Name der Datei, aus der das PPM gelesen werden soll
       * \return Zeiger auf das neu erzeugte, oder auf das übergebene Bild.
     */
      virtual ImageBuffer* read(ImageBuffer* image, std::istream& in) const
          throw(TribotsException);

      virtual std::string getDefEnding() const;
      virtual std::string getTypeId() const;
  };
};

#endif
