/**
 *  DirectionalCameraMapping.cpp
 *
 * Abbildungsklassen fuer gerichtete Kameras
 *
 */

#include "DirectionalCameraMapping.h"

#include <fstream>
#include "../../Structures/TribotsException.h"
#include "Image2WorldMapping.h"
#include "../../Structures/Journal.h"

// --------- ImageWorldDirectionalMapping: ----------------

using namespace Tribots;
using namespace std;

ImageWorldDirectionalMapping::ImageWorldDirectionalMapping(
    string filename,
    int croppedW,
    int croppedH,
    int xOffset,
    int yOffset)
{
  ifstream calibFile (filename.c_str());
  if (!calibFile || !optics.readParametersFromStream (calibFile))
    throw TribotsException ((string("ImageWorldDirectionalMapping::ImageWorldDirectionalMapping: Fehler beim Einlesen der Kameraparameter aus Datei \"") + filename + string("\"")).c_str());
  optics.shiftPrinciplePoint (-Vec(xOffset, yOffset));
  undistort=new UndistortionMapping (optics, croppedW, croppedH);
}

ImageWorldDirectionalMapping::~ImageWorldDirectionalMapping()
{
  delete undistort;
}

Vec3D ImageWorldDirectionalMapping::getOrigin() const throw () {
  return optics.cameraOrigin();
}

unsigned int ImageWorldDirectionalMapping::getWidth() const throw () {
  return undistort->getWidth();
}

unsigned int ImageWorldDirectionalMapping::getHeight() const throw () {
  return undistort->getHeight();
}

Line3D ImageWorldDirectionalMapping::map3D (const Vec& b) const
{
  Vec bu = undistort->map (b);
  return optics.inversePinholeMap (bu);
}

Vec ImageWorldDirectionalMapping::map(const Vec& b) const
{
  Vec bu = undistort->map (b);
  Angle azimuth;
  Angle inclination;
  optics.inversePinholeMap (azimuth, inclination, bu);
  if (inclination.get_rad_pi()>=0)
    // kein Schnittpunkt mit dem Boden, lege ihn ungefaehr nach Unendlich
    return 1e100*Vec::unit_vector (azimuth);
  return optics.inversePinholeMap (bu).intersectZPlane(0).toVec();
}


// --------- WorldImageDirectionalMapping: ----------------

WorldImageDirectionalMapping::WorldImageDirectionalMapping(
    string filename,
    int croppedW,
    int croppedH,
    int xOffset,
    int yOffset)
{
  ifstream calibFile (filename.c_str());
  if (!calibFile || !optics.readParametersFromStream (calibFile))
    throw TribotsException ((string("WorldImageDirectionalMapping::WorldImageDirectionalMapping: Fehler beim Einlesen der Kameraparameter aus Datei \"") + filename + string("\"")).c_str());
  optics.shiftPrinciplePoint (Vec(xOffset, yOffset));
  width=croppedW;
  height=croppedH;
  try{
    invalidArea=optics.nonObservableHalfplane ();
  }catch(invalid_argument&) {
    invalidArea=Halfplane (Vec(1e100, 0), Vec(1e100,0));
  }
}

WorldImageDirectionalMapping::~WorldImageDirectionalMapping() {;}

Vec WorldImageDirectionalMapping::map(const Vec& vec) const
{
  try{
    if (invalidArea.is_inside (vec))
      return Vec(-1,-1);
    return optics.map (Vec3D(vec.x, vec.y, 0));
  }catch(invalid_argument& e) {
    return Vec(-1,-1);
  }
}

Vec WorldImageDirectionalMapping::map3D(const Vec3D& vec) const
{
  try{
    return optics.map (vec);
  }catch(invalid_argument& e){
    return Vec(-1,-1);
  }
}

unsigned int WorldImageDirectionalMapping::getWidth() const throw () {
  return width;
}

unsigned int WorldImageDirectionalMapping::getHeight() const throw () {
  return height;
}
