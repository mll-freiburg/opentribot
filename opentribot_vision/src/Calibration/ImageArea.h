
#ifndef _Tribots_ImageArea_h_
#define _Tribots_ImageArea_h_

#include <deque>
#include "../Formation/RGBImage.h"
#include "ImageCoordinate.h"

namespace Tribots {

  /** Klasse, um Bildbereiche zu verwalten */
  class ImageArea {
  public:
    ImageArea () throw ();
    ImageArea (const ImageArea& a) throw ();

    const ImageArea& operator= (const ImageArea&) throw ();
    /** Vereinigung mehrerer Areas */
    const ImageArea& operator+= (const ImageArea&) throw ();
    /** Vereinigung mehrerer Areas */
    ImageArea operator+ (const ImageArea&) const throw ();

    /** liefert die Groesse eines Bereichs (Anzahl Pixel) */
    unsigned int size () const throw ();

    /** zeichnet den Bereich ins Bild im mit Farbe color ein */

    void draw (RGBImage& im, const RGBTuple& color) const throw ();
    /** einen zusammenhaengenden Bereich gleicher Farbe finden, ausgehend von seed.
        Alle Pixel dieses Bereichs auf Farbe replcolor abaendern. Suche auf das Rechteck
        [leftupper, rightlower] begrenzen (righlower und leftupper muessen gueltige Koordinaten
        innerhalb des Bildes sein, sonst Segmentation Fault) */

    void collectErase (RGBImage& im, const RGBTuple& replcolor, ImageCoordinate seed,
                      ImageCoordinate leftupper, ImageCoordinate rightlower) throw ();
    /** alle Bereiche mit Farbe color im Bildausschnitt [leftupper, rightlower] aus Bild im finden,
        aufsammeln und durch replcolor ersetzen, nur Bereiche mit mindestens minsize Pixeln
        beruecksichtigen */

    static std::deque<ImageArea> collectErase (RGBImage& im, const RGBTuple& color, const RGBTuple& replcolor,
                                              unsigned int minsize, ImageCoordinate leftupper, ImageCoordinate rightlower) throw ();

    /** pruefen, ob alle Pixel innerhalb eines Kreises um Mittelpunkt center mit Radius radius liegen */
    bool checkInsideCircle (ImageCoordinate center, double radius) const throw ();

    /** einen Kreis an die Area fitten, liefert true bei Erfolg und liefert in (cx, cy) den Kreismittelpunkt und in r den Kreisradius */
    bool fitCircle (double& cx, double& cy, double& r) const throw ();

    /** Pruefroutine, um festzustellen, ob der Bereich durch einen Ring mit Mittelpunkt (cx,cy)
          und Radius r beschrieben werden kann. Berechnet werden:
          minradius: kleinster Abstand eines Pixels zum Mittelpunkt,
          maxradius: groesster Abstand eines Pixels zum Mittelpunkt,
          maxangle: groesster Zwickel vom Mittelpunnkt aus gesehen, in dem keine Pixel liegen (in rad)
          sumangle: Summe der Winkel aller Zwickel ab 5 Grad, in denen keine Pixel liegen (in rad) */
    void ringCheck (double& minradius, double& maxradius, double& maxangle, double& sumangle, double cx, double cy, double r) const throw ();

  protected:
    std::deque<ImageCoordinate> pixels;    ///< die Pixel des Bereichs

  private:
    /** intern benoetigte Methode zu collectErase (implementiert ScanlineFill) */
    void collectErase (RGBImage& im, const RGBTuple& color, const RGBTuple& replcolor, int x1, int x2, int y, int dir, ImageCoordinate leftupper, ImageCoordinate rightlower) throw ();
  };

}

#endif
