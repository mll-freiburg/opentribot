#ifndef __DIRECTIONALCAMERAMAPPING_H_
#define __DIRECTIONALCAMERAMAPPING_H_
/*
 *  DirectionalCameraMapping.h
 *
 * Abbildungsklassen fuer gerichtete Kameras
 *
 */

#include "CoordinateMapping.h"
#include "../Calibration/CameraOptics.h"

namespace Tribots {
  class ImageWorldLUTMapping;


  /** Inverse Koordinatentransformation fuer eine perspektivische
      Kamera mit radialer und tangentialer Linserverzerrung*/
  class ImageWorldDirectionalMapping : public ImageWorldMapping {
  public:
    /** Argumente: filename=Kalibrierdatei mit Kameraparametern,
        croppedW, croppedH: Bildbreite und -hoehe,
        xOffset, yOffset: Wenn bei Kalibrierung eine andere Bildgroesse
        verwendet wurde wie jetzt der die Verschiebung des Hauptpunktes
        in Bildkoordinaten */
    ImageWorldDirectionalMapping(std::string filename,
                                                       int croppedW, int croppedH,
                                                       int xOffset = 0, int yOffset = 0);
    ~ImageWorldDirectionalMapping();

    Vec map(const Vec& vec) const;
    Line3D map3D(const Vec &pixel) const; 
    Vec3D getOrigin() const throw ();
    unsigned int getWidth() const throw ();
    unsigned int getHeight() const throw ();

  protected:
    CameraOptics optics;
    UndistortionMapping* undistort;
  };


  /** Koordinatentransformation fuer eine gerichtete Kamera;
      so lange diese Funktion nur selten benoetigt wird, ohne LUT */
  class WorldImageDirectionalMapping : public WorldImageMapping {
  public:
    WorldImageDirectionalMapping(std::string filename, int croppedW, int croppedH,
                                                       int xOffset = 0, int yOffset = 0);
    ~WorldImageDirectionalMapping();

    Vec map(const Vec& vec) const;
    Vec map3D(const Vec3D& vec) const; 
    unsigned int getWidth() const throw ();
    unsigned int getHeight() const throw ();

  protected:
    CameraOptics optics;
    Halfplane invalidArea;
    unsigned int width, height;
  };

}

#endif
