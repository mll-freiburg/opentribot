#ifndef _image_h_
#define _image_h_

#include "ImageBuffer.h"
#include "../PixelAnalysis/ColorClassifier.h"
#include "../../Fundamental/Time.h"

namespace Tribots {

  /**
   * Interface der bei den Tribots verwendeten Bilder. Das Interface 
   * bietet einige Pixelzugriffsmethoden, mit denen die Farbe der 
   * Pixel in verschiedenen Farbräumen gelesen oder gesetzt werden
   * kann. Dabei spielt das verwendete Format (YUV, RGB) des 
   * Bildpuffers beim Zugriff keine Rolle. Es gibt dabei verschiedene
   * Implementierungen, die unterschiedliche Formate für die interne
   * Repräsentierung des Bildes verwenden. Implementierungen
   * besitzt für die Speicherung des Bildes in der Regel keinen 
   * eigenen Speicherbereich sondern dienen lediglich als Wrapper
   * für eine Instanz vom Typ ImageBuffer. Passt dasFormat des
   * übergebenen ImageBuffers allerdings nicht zur ausgewählten
   * Implementierung von Image, so kann es sein, dass der 
   * ursprüngliche Puffer kopiert und konvertiert wird. In einem
   * solchen Fall übernimmt die Instanz vom Typ Image die Verwaltung
   * des neu belegten Speichers.
   *
   * Bei den unterschiedlichen Implementierungen sind die 
   * Zugriffsfunktionen unterschiedlich schnell. Es ist ev. vorteilhaft,
   * wenn die ausgewählte Implementierung (z.B. YUVImage) zu den
   * am häufigsten verwendeten Zugriffen (z.B. getPixelYUV) und zu der
   * verwendeten ImageSource (sollte einen Puffer im gleichen Format
   * liefern) passt. In der Dokumentation der verschiedenen 
   * Iplementierungen werden in der Regel die durschnittlichen 
   * Zugriffszeiten auf einer Referenzmaschine 
   * (P4 2,6 GHz, Hyperthreading) angegeben.
   */
  class Image {
  public:
    /**
     * erzeugt ein leeres Bild und verknüpft dieses mit einem 
     * Farbklassifizierer.
     *
     * \param classifier der zu verwendende ColorClassifier. Wird keiner
     *                   angegeben, so wird der DefaultClassifier 
     *                   verwendet, der immer 0 (COLOR_IGNORE) als Farbklasse
     *                   zurück gibt.
     */
    Image(const ColorClassifier* classifier= 0); 
    /**
     * zerstörrt ein Bild. Wurde bei der Konstruktion ein neuer ImageBuffer 
     * angelegt, wird dieser gelöscht.
     */
    virtual ~Image() {;}

    /**
     * Erzeugt eine exakte Kopie des Bildes mit einem eigenen Speicherbereich.
     */
    virtual Image* clone() const = 0;

    /**
     * liefert die Breite des Bildes
     */
    virtual int getWidth() const;
    /**
     * liefert die Höhe des Bildes
     */
    virtual int getHeight() const;

    /**
     * liefert den mit diesem Bild verknüpften Zeitstempel. Der Zeitstempel
     * wird bei der Konstruktion des Bildes auf die aktuelle Systemzeit 
     * gesetzt, kann aber auch nachträglich durch setTimestamp geändert werden.
     */
    virtual const Time& getTimestamp() const;
    /**
     * setzt den mit dem Bild verknüpften Zeitstempel auf die übergebene Zeit
     */
    virtual void setTimestamp(const Time& timestamp);

    /**
     * liest den Farbwert des Pixels (x,y) aus und schreibt ihn in das YUVTuple
     *
     * \param x x-Koordinate des auszulesenden Pixels
     * \param y y-Koordinate des auszulesenden Pixels
     * \param yuv Zeiger auf das YUVTuple, in das der Farbwert geschrieben 
     *            werden soll. Es ist nicht erlaubt, einen Nullzeiger 
     *            zu übergeben.
     */
    virtual void getPixelYUV (int x, int y, YUVTuple* yuv) const =0;
    /**
     * liest den Farbwert des Pixels (x,y) aus und schreibt ihn in das 
     * UYVYTuple
     *
     * \param x x-Koordinate des auszulesenden Pixels
     * \param y y-Koordinate des auszulesenden Pixels
     * \param uyvy Zeiger auf das UYVYTuple, in das der Farbwert geschrieben 
     *             werden soll. Es ist nicht erlaubt, einen Nullzeiger 
     *             zu übergeben.
     */
    virtual void getPixelUYVY(int x, int y, UYVYTuple* uyvy) const =0;
    /**
     * liest den Farbwert des Pixels (x,y) aus und schreibt ihn in das RGBTuple
     *
     * \param x x-Koordinate des auszulesenden Pixels
     * \param y y-Koordinate des auszulesenden Pixels
     * \param rgb Zeiger auf das RGB Tuple, in das der Farbwert geschrieben 
     *            werden soll. Es ist nicht erlaubt, einen Nullzeiger 
     *            zu übergeben.
     */
    virtual void getPixelRGB (int x, int y, RGBTuple* rgb) const =0;


    /**
     * setzt den Farbwert des Pixel (x,y) auf den übergebenen Wert
     *
     * \param x x-Koordinate des zu ändernden Pixels
     * \param y y-Koordinate des zu ändernden Pixels
     * \param yuv Farbe, auf die das Pixel gesetzt werden soll
     */
    virtual void setPixelYUV (int x, int y, const YUVTuple& yuv) =0;
    /**
     * setzt den Farbwert des Pixel (x,y) auf den übergebenen Wert
     *
     * \param x x-Koordinate des zu ändernden Pixels
     * \param y y-Koordinate des zu ändernden Pixels
     * \param uyvy Farbe, auf die das Pixel gesetzt werden soll
     */
    virtual void setPixelUYVY(int x, int y, const UYVYTuple& uyvy) =0;
    /**
     * setzt den Farbwert des Pixel (x,y) auf den übergebenen Wert
     *
     * \param x x-Koordinate des zu ändernden Pixels
     * \param y y-Koordinate des zu ändernden Pixels
     * \param rgb Farbe, auf die das Pixel gesetzt werden soll
     */
    virtual void setPixelRGB (int x, int y, const RGBTuple& rgb) =0;

    /**
     * liefert die Farbklasse des Pixels (x,y). Hierzu wird der 
     * mit diesem Bild verknüpfte ColorClassifier verwendet (siehe
     * Konstruktor). Wurde kein Farbklassifizierer angegeben, wird
     * als Default immer 0 (COLOR_IGNORE) zurück gegeben
     *
     * \attention Wenn interne Repräsentierung des Bildes und der 
     *            verwendete ColorClassifier zusammen passen
     *            (z.B. YUVImage und YUVLookupTable), ist dieser
     *            Aufruf in der Regel deutlich schneller als
     *            ein Umweg über 
     *            colorClassifier.lookup(getPixelXYZ(x,y)), da 
     *            Konvertierungen und Speicherkopien vermieden werden
     *            können.
     *
     * \param x x-Koordinate des auszulesenden Pixels
     * \param y y-Koordinate des auszulesenden Pixels
     * \returns Farbklasse des Pixels (siehe ColorClasses.h)
     */     
    virtual unsigned char getPixelClass(int x, int y) const =0;

    /**
     * liefert einen Wert (siehe ImageBuffer), der angibt, wie das Bild
     * intern repräsentiert wird.
     */
    virtual int getNativeFormat() const;

    /**
     * liefert einen Zeiger auf den derzeit mit dem Bild assoziierten 
     * ColorClassifier
     */
    virtual const ColorClassifier* getClassifier() const;   
    /**
     * ändert den derzeit mit dem Bild assoziierten ColorClassifier
     */
    virtual void setClassifier(const ColorClassifier* classifier);

    /**
     * erlaubt den direkten Zugriff auf den vom Image verwendeten Bildpuffer.
     *
     * \attention Änderungen an dem zurückgegeben ImageBuffer gelten auch für
     *            das Image und können hier zu Fehlern führen!
     */
    virtual ImageBuffer& getImageBuffer();
    virtual const ImageBuffer& getImageBuffer() const;

    /**
     * setzt alle Randpixel des Bildes auf die Farbe schwarz.
     */
    virtual void setBlackBorder();
    
    /**
     * setzt alle Randpixel des Bildes auf die Farbe weiss.
     */
    virtual void setWhiteBorder();

    /**
     * setzt alle Randpixel des Bildes auf die angegebene Farbe.
     */
    virtual void setBorder(const RGBTuple& rgb);

  protected:
    ImageBuffer buffer;                 ///< ImageBuffer mit dem Bild
    const ColorClassifier* classifier;  ///< der verwendete ColorClassifier

  private:
    /**
     * Keine Kopien erlaubt.
     */
    Image(const Image&) { ; }           
    /**
     * Keine Zuweisungen erlaubt.
     */
    Image& operator=(const Image&) {return *this; }
  };

}

#endif
