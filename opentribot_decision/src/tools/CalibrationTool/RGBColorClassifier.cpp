
#include "RGBColorClassifier.h"
#include "../../Fundamental/ConfigReader.h"
#include "../../ImageProcessing/Formation/PixelConversion.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

namespace {
  unsigned char returnval;  // gar nicht schoen diese Konstruktion aber aufgrund der Schnittstelle notwendig
}


RGBColorClassifier::RGBColorClassifier () {;}

RGBColorClassifier::~RGBColorClassifier () {;}

Tribots::ColorClassifier* RGBColorClassifier::create() const {
  RGBColorClassifier* newcl = new RGBColorClassifier;
  newcl->rmin = rmin;
  newcl->rmax = rmax;
  newcl->gmin = gmin;
  newcl->gmax = gmax;
  newcl->bmin = bmin;
  newcl->bmax = bmax;
  return newcl;
}

const unsigned char& RGBColorClassifier::lookup (const RGBTuple&  rgb)  const {
  bool yes;
  for (unsigned int i=0; i<rmin.size(); i++) {
    yes = (rmin[i] <= rgb.r) && (rgb.r <=rmax[i]);
    yes &= (gmin[i] <= rgb.g) && (rgb.g <=gmax[i]);
    yes &= (bmin[i] <= rgb.b) && (rgb.b <=bmax[i]);
    if (yes) {
      returnval = i+1;
      return returnval;
    }
  }
  returnval = 0;
  return returnval;
}

const unsigned char& RGBColorClassifier::lookup (const YUVTuple& yuv)  const {
  RGBTuple rgb;
  PixelConversion::convert (yuv, &rgb);
  return lookup (rgb);
}

const unsigned char& RGBColorClassifier::lookup (const UYVYTuple& uyuv, int pos) const {
  RGBTuple rgb;
  PixelConversion::convert (uyuv, &rgb, pos);
  return lookup (rgb);
}

void RGBColorClassifier::clear () {
  rmin.clear ();
  rmax.clear ();
  gmin.clear ();
  gmax.clear ();
  bmin.clear ();
  bmax.clear ();
}

void RGBColorClassifier::load (std::string filename) {
  clear();
  ConfigReader cfg;
  cfg.append_from_file (filename.c_str());
  vector<unsigned int> v;
  cfg.get ("RGBColorClassifier::settings", v);
  unsigned int numClass = v.size()/6;
  for (unsigned int i=0; i<numClass; i++) {
    rmin.push_back (v[6*i]);
    rmax.push_back (v[6*i]+1);
    gmin.push_back (v[6*i]+2);
    gmax.push_back (v[6*i]+3);
    bmin.push_back (v[6*i]+4);
    bmax.push_back (v[6*i]+5);
  }
}

void RGBColorClassifier::save (std::string filename) const {
  ofstream dest (filename.c_str());
  if (!dest) {
    cerr << "WARNUNG: RGBColorClassifier-Datei " << filename << " kann nicht geschrieben werden\n";
    return;
  }
  dest << "RGBColorClassifier::settings =";
  for (unsigned int i=0; i<rmin.size(); i++) {
    dest << ' ' << rmin[i] << ' ' << rmax[i] << ' ' << gmin[i] << ' ' << gmax[i] << ' ' << bmin[i] << ' ' << bmax[i];
  }
  dest << endl;
}

void RGBColorClassifier::createFromExamples (const std::vector<std::vector<Tribots::RGBTuple> >& sample) {
  clear ();
  unsigned int numClass = sample.size();
  rmin.assign (numClass, 255);
  rmax.assign (numClass, 0);
  gmin.assign (numClass, 255);
  gmax.assign (numClass, 0);
  bmin.assign (numClass, 255);
  bmax.assign (numClass, 0);

  for (unsigned int i=0; i<numClass; i++) {
    for (unsigned int j=0; j<sample[i].size(); j++) {
      if (sample[i][j].r<rmin[i]) rmin[i] = sample[i][j].r;
      if (sample[i][j].r>rmax[i]) rmax[i] = sample[i][j].r;
      if (sample[i][j].g<gmin[i]) gmin[i] = sample[i][j].g;
      if (sample[i][j].g>gmax[i]) gmax[i] = sample[i][j].g;
      if (sample[i][j].b<bmin[i]) bmin[i] = sample[i][j].b;
      if (sample[i][j].b>bmax[i]) bmax[i] = sample[i][j].b;
    }
    int ext = 8;
    rmin[i] = (rmin[i]>ext ? rmin[i]-ext : 0);
    rmax[i] = (rmax[i]<255-ext ? rmax[i]+ext : 255);
    gmin[i] = (gmin[i]>ext ? gmin[i]-ext : 0);
    gmax[i] = (gmax[i]<255-ext ? gmax[i]+ext : 255);
    bmin[i] = (bmin[i]>ext ? bmin[i]-ext : 0);
    bmax[i] = (bmax[i]<255-ext ? bmax[i]+ext : 255);
  }
}
