
#include "YUVLookupLearner.h"
#include <string>
#include <fstream>
#include <iostream>
#include "../../Structures/TribotsException.h"
#include "../Formation/PixelConversion.h"
#include <cstring>

namespace Tribots {

  using namespace std;

  YUVLookupLearner::YUVLookupLearner(int shiftY, int shiftU, int shiftV) 
    : ColorClassifier(), shiftY(shiftY), shiftU(shiftU), shiftV(shiftV),
      sizeY(256 >> shiftY), sizeU(256 >> shiftU), sizeV(256 >> shiftV),
      size(sizeY*sizeU*sizeV), lut(new unsigned char[size])
  {}

  YUVLookupLearner::~YUVLookupLearner()
  {
    delete [] lut;
  }
  
  const unsigned char&
  YUVLookupLearner::lookup(const RGBTuple& rgb) const
  {
    PixelConversion::convert(rgb, const_cast<YUVTuple*>(&yuvTmp));
    return lookup(yuvTmp);
  }
  const unsigned char&
  YUVLookupLearner::lookup(const UYVYTuple& uyvy, int pos) const
  {
    PixelConversion::convert(uyvy, const_cast<YUVTuple*>(&yuvTmp), pos);
    return lookup(yuvTmp);
  }
  const unsigned char&
  YUVLookupLearner::lookup(const YUVTuple& yuv) const
  {
    return lut[((yuv.y >> shiftY) * sizeU + 
		(yuv.u >> shiftU)) * sizeV +
	       (yuv.v >> shiftV)];
  }

  void 
  YUVLookupLearner::set(const RGBTuple&  rgb,  unsigned char c)
  {
    PixelConversion::convert(rgb, &yuvTmp);
    set(yuvTmp, c);
  }
  void 
  YUVLookupLearner::set(const UYVYTuple& uyvy, unsigned char c, int pos)
  {
    PixelConversion::convert(uyvy, &yuvTmp, pos);
    set(yuvTmp, c);
  }
  void
  YUVLookupLearner::set(const YUVTuple&  yuv,  unsigned char c)
  {
    lut[((yuv.y >> shiftY) * sizeU + 
	 (yuv.u >> shiftU)) * sizeV +
	(yuv.v >> shiftV)] = c;    
  }

  void 
  YUVLookupLearner::load(string filename)
  {
    int oldSize = size;// remember the size, to detect wheter or not it changes

    ifstream lutFile (filename.c_str(), ios::binary);
    if (! lutFile) {
      throw TribotsException((string(__FILE__)+string(": Could not open file ")+filename).c_str());
    }

    lutFile >> shiftY; 
    lutFile >> shiftU;
    lutFile >> shiftV; // this one reads the '\n' that follows the shiftV value

    sizeY = 256 >> shiftY;
    sizeU = 256 >> shiftU;
    sizeV = 256 >> shiftV;

    size = sizeY * sizeU * sizeV;

    if (oldSize != size) {              // size has been changed
      delete [] lut;                    // delete old and
      lut = new unsigned char[size];    // create new lookup table 
    }

    if (! lutFile.read((char*)lut, size)) {
      throw TribotsException((string(__FILE__)+string(": Could not read lookup table from file ")+filename).c_str());
    }
    lutFile.close();
  }

  
  void 
  YUVLookupLearner::save(string filename) const
  {
    ofstream lutFile (filename.c_str(), ios::binary);
    if (! lutFile) {
      throw TribotsException((string(__FILE__)+string(": Could not open file ")+filename).c_str());
    }
    lutFile << shiftY << ' ' << shiftU << ' ' << shiftV <<  '\n';

    if (! lutFile.write((char*)lut, size)) {
      throw TribotsException((string(__FILE__)+string(": Could not write lookup table to file ")+filename).c_str());
    }
    lutFile.close();
  }

  ColorClassifier* 
  YUVLookupLearner::create() const
  {
    return new YUVLookupLearner(shiftY, shiftU, shiftV);
  }

  void 
  YUVLookupLearner::fillFromClassifier(const ColorClassifier* cc)
  {
    int classCounter[256];    // 256 classes (unsigned char)
    YUVTuple yuv;
		
    for (int yQ = 0; yQ < sizeY; yQ++) {
      for (int uQ = 0; uQ < sizeU; uQ++) {
	for (int vQ = 0; vQ < sizeV; vQ++) { // for every entry in the lut


	  // check every yuv-tuple, that falls into the present quantizized
	  // entry of the lut

	  memset(classCounter, 0, sizeof(int) * 256);

	  for (int y = yQ << shiftY; y < (yQ+1) << shiftY; y++) {
	    for (int u = uQ << shiftU; u < (uQ+1) << shiftU; u++) {
	      for (int v = vQ << shiftV; v < (vQ+1) << shiftV; v++) {
		yuv.y = y; yuv.u = u; yuv.v = v;
		classCounter[cc->lookup(yuv)]++;
	      }
	    }
	  }
	  int maxID = 0;   // look for the most frequent class 
	  for (int i=1; i < 256; i++) {
	    if (classCounter[maxID] < classCounter[i]) { // more entries?
	      maxID = i; 
	    }
	  }
	  set(yuv, maxID); // since yuv is still _in_ the lut entry, this
                           // should do the necessary update (I hope...)
	}
      }
    }
  } 
}
