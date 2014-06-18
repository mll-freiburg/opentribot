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
       * kein Nullzeiger �bergeben, muss das �bergebene Image vom Typ RGBImage
       * sein und die richtige Gr��e haben. Wird ein Nullzeiger �bergeben,
       * legt read ein neues RGBImage passender Gr��e an, dass in den Bezitz des
       * Aufrufers �bergeht.
       * 
       * \param *image RGBImage mit passender Gr��e oder Nullzeiger
       * \param filename Name der Datei, aus der das PPM gelesen werden soll
       * \return Zeiger auf das neu erzeugte, oder auf das �bergebene Bild.
     */
      virtual ImageBuffer* read(ImageBuffer* image, std::istream& in) const
          throw(TribotsException);

      virtual std::string getDefEnding() const;
      virtual std::string getTypeId() const;
  };
};

#endif
