#ifndef _image2worldmapping_h_
#define _image2worldmapping_h_

#include <string>
#include <vector>
#include "CoordinateMapping.h"

namespace Tribots {

  /**
   * Bildet ganzzahlige Bildkoordinaten auf relative Weltkoordinaten ab.
   * Die Abbildung wird durch eine Nachschlagetabelle realisiert, in der die
   * korrekten Weltkoordinaten für jedes einzelne Pixel gespeichert sind.
   *
   * Die Nachschlagetabelle wird aus einer externen Datei eingelesen, die 
   * mit Hilfe eines Kalibrierungstools erzeugt wurde. Dabei hat
   * die Datei ein binäres Format. Vor dem Start der eigentlichen Daten
   * steht die Höhe und die Breite der Tabelle: "HÖHE BREITE\n". Die beiden int
   * werden durch EINE Leerstelle getrennt. Direkt nach der BREITE folgt
   * ein Zeilenunbruchzeichen '\n'. Hieran schließt sich der Puffer der 
   * Nachschlagetabelle an (Bildzeilenweise organisiert, ios::binary). 
   *
   * Die Struktur der binären Daten ist von der Implementierung des Vektors
   * (Vec.h) abhängig. Derzeit bilden je 16 Byte einen Block. Ein Block
   * entspricht hierbei einem Eintrag in der Nachschlagetabelle, bzw.
   * einem Object vom Typ Vec. Jeder Block besteht aus zwei double Werten
   * für x- (die ersten 8 Byte) und y-Koordiante (die letzten 8 Byte).
   *
   * Für die 3D Mapping funktion schließt sich an die definition des Lookup-
   * tables die Position des Kameraursprngs in Weltkoordinaten an. Dieser ist
   * vom Typ Vec3D (12 Byte).
   *
   * map3D() liefert eine Gerade, die den Kameraurpsrung mit der z=0-Ebene 
   * verbindet.
   *
   */
  class ImageWorldLUTMapping : public ImageWorldMapping {
  public:
    ImageWorldLUTMapping(std::string filename);
    ImageWorldLUTMapping(const ImageWorldMapping& map);  ///< erzeugt LUT aus map
    ImageWorldLUTMapping(int width, int height, Vec3D origin=Vec3D(0.,0.,0.));
    virtual ~ImageWorldLUTMapping();

    /**
      * Lädt eine Nachschlagetabelle aus einer Datei. Die Datei enthält
     * Informationen über die robozentrischen Weltkoordinaten für jedes
     * einzelne Pixel. Daher muss die Größe der in der Datei enthaltenen
     * Nachschlagetabelle (siehe getWidth und getHeight Methoden) zu der
     * aktuell verwendeten Bildgröße passen!
     */
    virtual void load(std::string filename);

    /**
      * Speichert die aktuelle Nachschlagetabelle in einer Datei.
     */
    virtual void save(std::string filename) const;

    /** Gets the Origin of the Camera in World Coordinates.
        DON'T RELY ON IT. MAYBE IT IS NOT SET.
      */
    Vec3D getOrigin() const { return origin; } 

    /**
      * Bildet einen Vektor mit Bildkoordinaten in relative Weltkoordinaten ab.
     *
     * \param  vec Vektor mit Bildkoordinaten (Pixel)
     * \returns    Vektor mit relativen Weltkoordinaten
     */
    Vec map(const Vec& vec) const;

    /** Returns the Line3D from the Camera origin to the z=0 plane,
      *  to which the given pixel projects.
      *
      * \param  pixel Vektor mit Bildkoordinaten (Pixel)
      * \returns      Line3D - Gerade durch Kameraursprung und x-y-plane
      */
    Line3D map3D(const Vec &pixel) const;

    /**
      * Setzt den relativen Weltkoordinatenvektor für das Pixel an der 
     * Stelle (x,y). Wird zum erzeugen der Nachschlagetabelle in der 
     * Kalibrierungsphase benötigt.
     */
    virtual void set(int x, int y, const Vec& vec);

    /**
     * Setzt die 3D-Linie durch den Kameraursprung und den Bodenschnittpunkt fŸr das Pixel
     * an der Stelle (x,y). Zum Erzeugen der 3D-LUT.
     */
    virtual void set3D(int x, int y, const Line3D& vec);

    /** Erzeugt die Nachschlagetabelle durch Auswerten von map */
    virtual void convert(const ImageWorldMapping& map);

    virtual unsigned int getWidth() const throw () { return width; };
    virtual unsigned int getHeight() const throw () { return height; }

  protected:
    void resize (int nwidth, int nheight);
    Vec3D origin; ///< coordinates of the camera origin; DON'T RELY ON IT. MAYBE IT IS NOT SET.
    Vec* lut;   ///< holds the relative world coords for every pixel
    Line3D* lut3d; ///< holds the relative world coords in 3D
                  // incorporating special handling of mappings (e.g. above-horizon etc)
    int width;
    int height;
    int lutSize;
  };

}

#endif
