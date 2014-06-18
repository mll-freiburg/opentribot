
#ifndef _Tribots_CoordinateMapping_h_
#define _Tribots_CoordinateMapping_h_

#include "../../Fundamental/Vec.h"
#include "../../Fundamental/Vec3D.h"
#include "../../Fundamental/geometry3D.h"

namespace Tribots {

  /** abstrakte Klasse, die allgemeine Koordinaten-Transformationen (nicht notwendigerweise affine) modelliert */
  class CoordinateMapping {
  public:
    virtual ~CoordinateMapping() {};

    virtual unsigned int getWidth() const throw () =0;  ///< Maximalbreite (z.B. bei LUTs)
    virtual unsigned int getHeight() const throw () =0;  ///< Maximalhoehe (z.B. bei LUTs)

    /** Punkt-zu-Punkt-Abbildung */
    virtual Vec map(const Vec&) const =0;
    virtual Vec map(int x, int y) const { return map(Vec(x, y)); }
    virtual Vec map(double x, double y) const { return map(Vec(x,y)); }
  };

  /** abstrakte Klasse als Elternklasse fuer alle Welt->Bild-Abbildungen */
  class WorldImageMapping : public CoordinateMapping {
  public:
    /**
     * Maps back a 3D position into image coordinates. 
     *
     * \param vec 3D world coordinates that should be mapped back to image
     *            coordinates
     */
    virtual Vec map3D(const Vec3D& vec) const=0; 
    virtual Vec map3D(double x, double y, double z) const { return map3D(Vec3D(x,y,z)); }
  };

  /** abstrakte Klasse als Elternklasse fuer alle Bild->Welt-Abbildungen */
  class ImageWorldMapping : public CoordinateMapping {
  public:
    /**
    * Returns the Line3D through the Camera origin that is projected onto 
    * the pixel x,y.
    *
    * \param  x X-Koordinate des Pixels
    * \param  y Y-Koordinate des Pixels
    * \returns      Line3D - Gerade durch Kameraursprung und x-y-plane
    */
    virtual Line3D map3D(int x, int y) const { return map3D(Vec(x,y)); }

    /**
    * Returns the Line3D through the Camera origin that is projected onto 
    * the pixel pixel.
    *
    * \param  pixel Vektor mit Bildkoordinaten (Pixel)
    * \returns      Line3D - Gerade durch Kameraursprung und x-y-plane
    */
    virtual Line3D map3D(const Vec &pixel) const=0;
  };

}

#endif
