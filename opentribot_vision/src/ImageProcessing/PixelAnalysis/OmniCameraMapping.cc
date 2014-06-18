/*
 *  OmniCameraMapping.cpp
 *  robotcontrol
 *
 *  Created by Sascha Lange on 19.03.07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include "OmniCameraMapping.h"
#include <fstream>
#include "../../Structures/TribotsException.h"
#include "../../Structures/Journal.h"
#include <sstream>

// --------- ImageWorldOmniMapping: ----------------

using namespace Tribots;
using namespace std;

ImageWorldOmniMapping::ImageWorldOmniMapping(string filename,
                                             Vec3D cameraOrigin,
                                             int externCenterX, int externCenterY)
: cameraOrigin(cameraOrigin)
{
  ifstream file(filename.c_str());
  if (!file) {
    throw TribotsException(__FILE__": Could not open distance file.");
  }
  
  // read image dimensions and center
  int centerX, centerY;
  file >> width >> height;
  file >> centerX >> centerY;
  middle = Vec(externCenterX > 0 ? externCenterX : centerX,
               externCenterY > 0 ? externCenterY : centerY);
  
  // read angle shift
  file >> shift;
  
  // read number of markers
  int numMarkers;
  file >> numMarkers;
  
  // read real world distances of markers (mm)
  for (int i=0; i < numMarkers; i++) {
    double distance;
    file >> distance;
    realDistances.push_back(distance);
  }
  
  // initialize imageDistances ( 360>>shift list of numMarkers entries each )
  imageDistances = vector<vector<double> >(360>>shift, 
                                           vector<double>(numMarkers));
  
  // initialize angleCorrections
  angleCorrections = vector<double> (360>>shift);
  
  while (!file.eof()) {
    int angle;
    double angleCorrection;
    file >> angle >> angleCorrection;
    if (file.eof()) { // end of file (or error)
      break;
    }
    
    angleCorrections[angle] = angleCorrection;
    
    for (int i=0; i < numMarkers; i++) {
      double distance;
      file >> distance;
      if (file.eof()) {
        throw TribotsException(__FILE__
                               ": File ended although another distance "
                               "entry was expected.");
      }
      imageDistances[angle][i] = distance;
    }
  }
  file.close();
}

ImageWorldOmniMapping::~ImageWorldOmniMapping()
{
}


Line3D
ImageWorldOmniMapping::map3D(const Vec& vec) const
{
  return Line3D(cameraOrigin, map(vec));
}

Vec ImageWorldOmniMapping::map(const Vec& vec) const
{
  if (vec.x != vec.x || vec.y != vec.y) {
    JERROR("received nan vector in IWOM::map()");
 // ARGH! Das geht im Moment nicht, da es auch im colorTool verwendet
 //       wird und das hat kein WorldModel und damit auch kein LOUT
 //       definiert.
    return Vec(1e100, 1e100);
  }
  Vec to = vec-middle;
  double toLength = to.length();
  int degrees = static_cast<int>( to.angle().get_deg() ) >> shift;
  
  // step through the list of imageDistances and find first entry,
  // that is bigger than the distance of the given vector (look at i+1).
  // if the position is size()-2, than assume last entry (i+1) to be bigger
  // than the length of the given vector (because last entry should be 
  // infinity per definition (biggest entry, that could be queried) ).
  // the first entry is assumed to be smaller than every query.
  
  for (unsigned int i = 0; i < imageDistances[degrees].size()-1; i++) {
    if (toLength < imageDistances[degrees][i+1] || // between i and i+1
        i == imageDistances[degrees].size()-2) {   // last marker:=infinity
      
      // linear interpolation of length in real world
      double steigung =   // deltaY / deltaX  ; y1,y2:real, x1,x2:image
      (realDistances[i+1] - realDistances[i]) /
      (imageDistances[degrees][i+1] - imageDistances[degrees][i]);
      
      to = to.normalize();
      to *= realDistances[i] + steigung * //y=x1 + steigung * (x-x1)
        (toLength - imageDistances[degrees][i]);
      
      // correct orientation
      to = to.rotate(Angle::deg_angle(angleCorrections[degrees]));
      break;
    }
  }
  // assertion: to has been transformed ( if is true for i=size()-2 )
  if (to.x!=to.x || to.y!=to.y) {
    JERROR("NAN_ERROR: return nan in ImageWorldOmniMapping::map().");
  }
  return to;
}


// WORLD -> IMAGE //////////////////////////////////////////////////////////////

WorldImageOmniMapping::WorldImageOmniMapping(string filename, Vec3D origin,
                                             int externCenterX, int externCenterY) : origin(origin)
{
  ifstream file(filename.c_str());
  if (!file) {
    throw TribotsException(__FILE__": WorldImageOmniMapping:"
                           " Could not open distance file.");
  }
  
  // read image dimensions and center
  int centerX, centerY;
  file >> width >> height;
  file >> centerX >> centerY;
  middle = Vec(externCenterX > 0 ? externCenterX : centerX,
               externCenterY > 0 ? externCenterY : centerY);
  
  // read angle shift
  file >> shift;
  
  // read number of markers
  int numMarkers;
  file >> numMarkers;
  
  // read real world distances of markers (mm)
  for (int i=0; i < numMarkers; i++) {
    double distance;
    file >> distance;
    realDistances.push_back(distance);
  }
  
  // initialize imageDistances ( 360>>shift list of numMarkers entries each )
  imageDistances = vector<vector<double> >(360>>shift, 
                                           vector<double>(numMarkers));
  // initialize angleCorrections
  angleCorrections = vector<double> (360>>shift);
  
  while (!file.eof()) {
    int angle;
    double angleCorrection;
    file >> angle >> angleCorrection;
    if (file.eof()) { // end of file (or error)
      break;
    }
    
    angleCorrections[angle] = angleCorrection;
    
    for (int i=0; i < numMarkers; i++) {
      double distance;
      file >> distance;
      if (file.eof()) {
        throw TribotsException(__FILE__
                               ": File ended although another distance "
                               "entry was expected.");
      }
      imageDistances[angle][i] = distance;
    }
  }
  file.close();
}

WorldImageOmniMapping::~WorldImageOmniMapping()
{
}


Vec WorldImageOmniMapping::map3D(const Vec3D& vec) const
{
  Vec3D point = Line3D(vec,origin).intersectZPlane(0.);
  return this->map(point.toVec());
}


Vec WorldImageOmniMapping::map(const Vec& vec) const
{
  Vec to = vec;
  double toLength = vec.length();
  int degrees = static_cast<int>( to.angle().get_deg() ) >> shift; 
  // \todo : apply angleCorrections here!
  
  
  // step through the list of realDistances and find first entry,
  // that is bigger than the distance of the given vector (look at i+1).
  // if the position is size()-2, than assume last entry (i+1) to be bigger
  // than the length of the given vector (because last entry should be 
  // infinity per definition (biggest entry, that could be queried) ).
  // the first entry is assumed to be the smaller than every query.
  
  for (unsigned int i = 0; i < realDistances.size()-1; i++) {
    if (toLength < realDistances[i+1] || // between i and i+1
        i == realDistances.size()-2) {   // last marker:=infinity
      
      // linear interpolation of length in real world
      double steigung =   // deltaY / deltaX  ; y1,y2:real, x1,x2:image
      (imageDistances[degrees][i+1] - imageDistances[degrees][i]) /
      (realDistances[i+1] - realDistances[i]);
      
      to = to.normalize();
      to *= imageDistances[degrees][i] + steigung* //y=x1 + steigung * (x-x1)
        (toLength - realDistances[i]);
      
      break;
    }
  }
  // assertion: to has been transformed ( if is true for i=size()-2 )
  if (to.x!=to.x || to.y!=to.y) {
    JERROR("NAN_ERROR: return nan in WorldImageOmniMapping::map().");
  }
  return to + middle;
}

unsigned int ImageWorldOmniMapping::getWidth() const throw () { return width; }
unsigned int ImageWorldOmniMapping::getHeight() const throw () { return height; }
unsigned int WorldImageOmniMapping::getWidth() const throw () { return width; }
unsigned int WorldImageOmniMapping::getHeight() const throw () { return height; }
