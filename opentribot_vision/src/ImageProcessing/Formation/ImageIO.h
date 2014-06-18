#ifndef _ImageIO_h_
#define _ImageIO_h_

#include "ImageBuffer.h"
#include "../../Structures/TribotsException.h"
#include <string>
#include <iostream>

namespace Tribots {

  class ImageIO {
  public:
    virtual ~ImageIO() {};
    /** schreibt ein Bild (arg1) nach Datei (arg2) */
    virtual void write(const ImageBuffer& image, const std::string& filename) const
      throw(TribotsException);
    /** schreibt ein Bild (arg1) nach Stream (arg2) */
    virtual void write(const ImageBuffer& image, std::ostream& out) const
      throw(TribotsException) =0;

    /**
     * Liest ein Bild ein. Wenn image != NULL ist, wird versucht, das Bild
     * in den Puffer des übergebenen Bildes zu schreiben. Dies schlägt mit
     * einer Exception fehl, wenn der Puffer nicht die passende Größe und
     * das passende Format hat.
     *
     * Wenn man die Ausmaße des einzulesenden Bildes nicht kennt, übergibt
     * man am besten einen Nullzeiger. Dann wird ein neues, passendes Image
     * angelegt.
     *
     * \param *image Zeiger auf ein Bild, dessen Puffer gefüllt werden soll.
     *               Schlägt fehlt, wenn der Puffer das falsche Format oder die
     *               falsche Größe hat.
     * \param filename Name der Datei, aus der gelesen werden soll.
     * \return Zeiger auf das neu erzeugte, oder auf das übergebene Bild.
     */
    virtual ImageBuffer* read(ImageBuffer* image, const std::string& filename) const
      throw(TribotsException);
    /** liest ein Bild (Return=arg1) aus Stream (arg2) */
    virtual ImageBuffer* read(ImageBuffer* image, std::istream& in) const
      throw(TribotsException) =0;

    /** liefert z.B. .ppm oder .jpg je nach Typ */
    virtual std::string getDefEnding() const { return ""; }
    /** liefert z.B. PPM, JPEG je nach Typ */
    virtual std::string getTypeId() const { return ""; }

  };

}

#endif
