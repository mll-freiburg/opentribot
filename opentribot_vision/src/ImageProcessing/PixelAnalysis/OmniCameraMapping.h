#ifndef _OmniCameraMapping_h_
#define _OmniCameraMapping_h_

/*
 *  OmniCameraMapping.h
 *  robotcontrol
 *
 *  Created by Sascha Lange on 19.03.07.
 *  Copyright 2007 University of Osnabrueck. All rights reserved.
 *
 */

#include "CoordinateMapping.h"

namespace Tribots {

  /** Koordinatentransformation fuer eine Omni-Kamera
   * Arbeitet mit Markerfiles
   */
  class ImageWorldOmniMapping : public ImageWorldMapping {
  public:
    ImageWorldOmniMapping(std::string filename,
                          Vec3D cameraOrigin=Vec3D(0.,0.,710.),
                          int centerX=-1, int centerY=-1);
    ~ImageWorldOmniMapping();

    Vec map(const Vec& vec) const;
    Line3D map3D(const Vec &pixel) const;
    unsigned int getWidth() const throw ();
    unsigned int getHeight() const throw ();

    virtual Vec getImageCenter() { return middle; }

  protected:
    int width, height;
    Vec middle;
    int shift;

    Vec3D cameraOrigin;

    std::vector<double> realDistances;
    std::vector<std::vector< double> > imageDistances;
    std::vector<double> angleCorrections;
  };


  /**
    * Bildet egozentrische Weltkoordinaten in Bildkoorinaten ab.
    * Liest aus einem Markerfile.
    */
  class WorldImageOmniMapping : public WorldImageMapping {
  public:
    WorldImageOmniMapping(std::string filename, 
                          Vec3D cameraOrigin=Vec3D(0.,0.,710.),
                          int centerX=-1, int centerY=-1);
    ~WorldImageOmniMapping();

    /** maps back a 3D position into image coordinates given cameraHeight */
    Vec map3D(const Vec3D& vec) const;
    Vec map(const Vec& vec) const;
    unsigned int getWidth() const throw ();
    unsigned int getHeight() const throw ();

  protected:
    Vec3D origin;

    int width, height;
    Vec middle;
    int shift;

    std::vector<double> realDistances;
    std::vector<std::vector< double> > imageDistances;
    std::vector<double> angleCorrections;
  };

}


#endif
