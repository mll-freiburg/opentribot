#ifndef _UYVYImage_h_
#define _UYVYImage_h_

#include "Image.h"

namespace Tribots {

  /**
   * Implementierung des Image Interfaces. UYVYImage verwendet den 
   * UYVY Farbraum zur Repräsentierung der Bilddaten. Hat der übergebene
   * ImageBuffer ein anderes Format, so wird versucht diesen bei der
   * Konstruktion in das UYVY Format zu konvertieren. 
   */
  class UYVYImage : public Image
  {
  public:
    /**
     * Wickelt ein Image um den übergebenen ImageBuffer. Verwendet der
     * ImageBuffer ein anderes Format als das UYVY Format, wird dieser
     * kopiert und konvertiert. Wird in der zentralen
     * Registrierung für Konvertierungsmethoden (ImageConversionRegistry)
     * keine passende Konvertierungsmethode gefunden, schlägt die 
     * Konstruktion mit einer Exception fehl.
     *
     * \param buffer die Bilddaten
     */
    UYVYImage(const ImageBuffer& buffer);
    UYVYImage(int width, int height);
    virtual ~UYVYImage();

    virtual Image* clone() const;

    virtual void getPixelYUV (int x, int y, YUVTuple* yuv) const;
    virtual void getPixelUYVY(int x, int y, UYVYTuple* uyvy) const;
    virtual void getPixelRGB (int x, int y, RGBTuple* rgb) const;

    virtual void setPixelYUV (int x, int y, const YUVTuple& yuv);
    virtual void setPixelUYVY(int x, int y, const UYVYTuple& uyvy);
    virtual void setPixelRGB (int x, int y, const RGBTuple& rgb);

    virtual unsigned char getPixelClass(int x, int y) const;

  protected:
    bool controlsBuffer; ///< true, wenn der interne Puffer der Instanz gehört

  };

}

#endif
