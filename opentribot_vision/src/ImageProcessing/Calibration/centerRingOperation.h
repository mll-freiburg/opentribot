
#ifndef _Tribots_centerRingOperation_h_
#define _Tribots_centerRingOperation_h_

#include "../Formation/RGBImage.h"

namespace Tribots {

  /** Funktion, um den gelben Ring um die Kameralinse im Bild zu finden
      (ccx, ccy), ccrmin, ccrmax: Mittelpunkt, minimaler und maximaler Radius des Rings (Rueckgabewerte)
      src: Bild
      dest: Rueckgabebild, muss die selbe Groesse wie src besitzen
      debug: soll eine debug-Ausgabe (in dest, sowie textuell) erfolgen?

      Algorithmenbeschreibung (Farbangabe entspricht Farben auf Debug-Bild):
      -# alle Pixel im Bild markieren, die gelblich erscheinen
      -# zusammenhaengende Pixel in Bereichen gruppieren
      -# kleine Bereiche (blau) oder Bereiche ausserhalb der Bildmitte (gruen) entfernen
      -# einen Kreis auf den Bereich fitten
      -# pruefen, ob der Bereich den Mittelpunkt des gefitteten Kreises umfasst. Wenn nein, Bereich entfernen (cyan)
      -# pruefen, ob der Ring hinreichend gut ausgefuellt ist. Wenn nein, Bereich entfernen (orange)
      -# pruefen, ob die Groesse des Rings (innerer und ausserer Radius) passt. Wenn nein, Bereich entfernen (magenta)
      -# anhand der Groesse besten Bereich auswaehlen und zurueckgeben
      -# wenn kein Bereich passt, dann versuchen, gut zueinander passende Bereiche zusammenzufassen und zusammengebaute Bereiche pruefen
      -# zum Ring beitragende Bereiche werden rot dargestellt

      Debug-Ausgabe:
      - Bereichs-ID
      - Pixelanzahl
      - Ringmittelpunkt
      - Ringradius, kleinster Radius, groesster Radius
      - umgibt der Ring den Mittelpunkt?
      - ein abschliessender Stern bedeutet, der Bereich ist Teil des Rings
  */
  bool findCenterRing (double& ccx, double& ccy, double& ccrmin, double& ccrmax, RGBImage& dest, const Image& src, bool debug =false);

  /** wie zuvor, aber zusaetzlich wird ein Bildausschnitt (x0, y0, x1, y1) definiert, auf den die Suche beschraenkt werden soll */
  bool findCenterRing (double& ccx, double& ccy, double& ccrmin, double& ccrmax, RGBImage& dest, const Image& src,
                       unsigned int x0, unsigned int y0, unsigned x1, unsigned int y1, bool debug =false);

  /** die Koordinaten die Grenzen des Weissabgleichsbereichs (x1,y1,x2,y2) aus den Ringkoordinaten (ccx, ccy, ccrmin, ccrmax) bestimmen */
  void determineBalanceArea (unsigned int& x1, unsigned int& y1, unsigned int& x2, unsigned int& y2, double ccx, double ccy, double ccrmin, double ccrmax) throw ();
}

#endif
