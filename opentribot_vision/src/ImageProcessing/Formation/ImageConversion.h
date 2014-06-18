#ifndef _imageconversion_h_
#define _imageconversion_h_


#include "../../Structures/TribotsException.h"
#include "ImageBuffer.h"
#include <map>

namespace Tribots {

  /**
   * Schnittstelle für Konvertierungsfunktionen. Ein ImageConverter   
   * kann einen ImageBuffer genau eines Formats in einen ImageBuffer 
   * eines bestimmten anderen Formats (möglichst effizient) umwandeln. 
   * Für jede mögliche Formatkombination müssen getrennte Konverter 
   * implementiert werden.
   */
  class ImageConverter
  {
  public:
    /**
     * Konstruktor. Die Formate werden von den einzelnen Implementierungen
     * passend gesetzt.
     */
    ImageConverter(int srcFormat, int dstFormat);
    virtual ~ImageConverter () {;}

    /**
     * Konvertiert und schreibt den ImageBuffer src in den ImageBuffer dst.
     * Ist das Format von src und dst gleich, wird der Puffer nur kopiert.
     * Werden die Formate von src und dst nicht von dem Konverter unterstützt
     * wird eine TribotsException geworfen.
     */
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException)=0;
    /**
     * liefert das Format, das der Konverter auf Seiten der Quelle unterstützt
     */
    int getSourceFormat() { return srcFormat; }
    /**
     * liefert das Format, das der Konverter auf Seiten des Ziels unterstützt.
     */
    int getDestinationFormat() { return dstFormat; }

  protected:
    int srcFormat;   ///< Unterstütztes Format der Quelle
    int dstFormat;   ///< Unterstütztes Format des Ziels
  };

  /**
   * Zentrale, als Singleton implementierte Registrierung, bei der alle 
   * vorhandenen Konvertierungsfunktionen für ImageBuffer registriert und
   * unter Angabe des gewünschten Quellen- und Zielformats nachgeschlagen
   * werden können.
   */
  class ImageConversionRegistry
  {
  public:
    /**
     * liefert einen Zeiger auf die zentrale Registrierung.
     *
     * \attention Kein delete auf den übergebenen Zeiger ausführen!
     */
    static ImageConversionRegistry* getInstance();
    /**
     * Destruktor.
     */
    ~ImageConversionRegistry();

    /**
     * schlägt einen Konverter nach. Ist keine passende Implementierung
     * registriert, wird eine Exception geworfen.
     *
     * \param Format der Quelle
     * \param Format des Ziels
     */
    const ImageConverter* getConverter(int srcFormat, int dstFormat) 
      throw (TribotsException);
    /**
     * registriert eine Implementierung. Wurde vorher bereits eine
     * andere Implementierung für die gleiche Formatumwandlung registriert,
     * wird diese überschrieben.
     *
     * \todo Beim Überschreiben muss die alte Implementierung auch 
     *       gelöscht werden!!!
     */
    void registerConverter(ImageConverter* converter);

  protected:
    std::map<int, std::map<int, ImageConverter*> > converterMap;
    ImageConverter* copy;

    static ImageConversionRegistry* singleton;
    
    /**
     * geschützter Konstruktor, Zugriff auf das Singleton nur mittels 
     * getInstance()
     */
    ImageConversionRegistry();    
  };

  /**
   * Kopiert den Puffer bei gleichem src und dst Format.
   */
  class CopyConverter : public ImageConverter
  {
  public:
    CopyConverter();
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException);
  };


  /**
   * Wandelt Puffer von YUV nach RGB um.
   */
  class YUV2RGB : public ImageConverter
  {
  public:
    YUV2RGB();
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException);
  };

  /**
   * Wandelt Puffer von RGB nach YUV um.
   */
  class RGB2YUV : public ImageConverter
  {
  public:
    RGB2YUV();
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException);
  };

  /**
   * Wandelt Puffer von YUV (wobei die Kanäle nach dem Schema UYV angeordnet 
   * sind) nach RGB um.
   */
  class UYV2RGB : public ImageConverter
  {
  public:
    UYV2RGB();
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException);
  };

  /**
   * Wandelt Puffer von YUV 4:2:2 nach YUV um.
   */
  class UYVY2YUV : public ImageConverter
  {
  public:
    UYVY2YUV();
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException);
  };

  /**
   * Wandelt Puffer von YUV 4:2:2 nach RGB um.
   */
  class UYVY2RGB : public ImageConverter
  {
  public:
    UYVY2RGB();
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException);
  };


  /**
   * Wandelt Puffer von YUV 4:1:1 nach YUV um.
   */
  class YUV4112YUV : public ImageConverter
  {
  public:
    YUV4112YUV();
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException);
  };


  /**
   * Wandelt Puffer von YUV 4:1:1 nach RGB um.
   */
  class YUV4112RGB : public ImageConverter
  {
  public:
    YUV4112RGB();
    virtual void operator() (const ImageBuffer& src, ImageBuffer& dst) const
      throw (TribotsException);
  };
}


#endif
