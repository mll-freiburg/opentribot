#include "ImageVarianceFilter.h"
#include "../../ImageProcessing/Formation/RGBImage.h"

#include <iostream>
#include <cmath>

namespace Tribots {

  using namespace std;

  void 
  ImageVarianceFilter::initialize(const Image& image)
  {
    initialize(image.getWidth(), image.getHeight());
  }

  ImageDistanceFilter::ImageDistanceFilter()
    : ImageVarianceFilter(), initialized(false), numImagesAdded(0), diff(0), 
      tmp(0), lastImage(0)
  {}

  ImageDistanceFilter::~ImageDistanceFilter()
  {
    if (diff) delete [] diff;
    if (tmp) delete [] tmp;
    if (lastImage) delete lastImage;
  }

  int
  ImageDistanceFilter::getNumImagesAdded() const
  {
    return numImagesAdded;
  }

  void
  ImageDistanceFilter::initialize(int w, int h)
  {
    if (diff) delete [] diff;
    if (tmp) delete [] tmp;
    if (lastImage) delete lastImage;
  
    diff = new double[w*h];
    tmp = new double[w*h];
    lastImage = 0;

    numImagesAdded = 0;  
    this->w = w;
    this->h = h;
  }

  /**
   * calculates the euclidian difference (to the power of two) between the u 
   * and v values of two YUVTuples
   */
  static double
  uvDistanceSquare(const YUVTuple& yuv1, const YUVTuple& yuv2) {
    return 
      pow(yuv1.u-yuv2.u, 2.) +
      pow(yuv1.v-yuv2.v, 2.);
  }

  static double
  uvDistance(const YUVTuple& yuv1, const YUVTuple& yuv2) {
    return sqrt(uvDistanceSquare(yuv1, yuv2));
  }


  /**
   * calculates the euclidian difference (to the power of two) between two 
   * YUVTuples
   */
  static double
  yuvDistanceSquare(const YUVTuple& yuv1, const YUVTuple& yuv2) {
    return 
      pow(yuv1.y-yuv2.y, 2.) +
      pow(yuv1.u-yuv2.u, 2.) +
      pow(yuv1.v-yuv2.v, 2.);
  }

  static double
  yuvDistance(const YUVTuple& yuv1, const YUVTuple& yuv2) {
    return sqrt(yuvDistanceSquare(yuv1, yuv2));
  }

  
  void
  ImageDistanceFilter::add(const Image& image)
  {
    if (image.getWidth() != w || image.getHeight() != h) {
      throw TribotsException(__FILE__ "Image format does not match.");
    }
    if (lastImage == 0) {          // remember first frame
      lastImage = image.clone();
      
    }
    else {                         // calculate pixelwise difference 
      YUVTuple yuv1, yuv2;         // between images t and t-1
      for (int x=0; x < w; x++) {  
	for (int y=0; y < h; y++) {
	  lastImage->getPixelYUV(x,y, &yuv1);
	  image.getPixelYUV(x,y, &yuv2);
	  diff[x+y*w] += uvDistanceSquare(yuv1, yuv2);
	}
      }
      numImagesAdded++;
    }
  }

  Image*
  ImageDistanceFilter::createMask(double threshold)
  {
    if (numImagesAdded <= 0) {
      throw TribotsException(__FILE__ "Can not create mask, no images added.");
    }
    if (lastImage == 0) {        // assertion 
      throw TribotsException(__FILE__ "FATAL: lastImage==0, although "
			     "numImagesAdded > 0 !");
    }
    Image* image = new RGBImage(w,h);
    RGBTuple white = { 255, 255, 255 };

    for (int x=0; x < w; x++) {
      for (int y=0; y < h; y++) {          // calculate mean
	tmp[x+y*w] = sqrt(diff[x+y*w] / numImagesAdded);
	if (tmp[x+y*w] > threshold) {
	  image->setPixelRGB(x, y, white);
	}
      }
    }
    return image;
  }




  // ////////////// maxDistance ///////////////////////////////////////////////

  ImageMaxDistanceFilter::ImageMaxDistanceFilter()
    : ImageVarianceFilter(), initialized(false), numImagesAdded(0), maxDiff(0),
      lastImage(0)
  {}

  ImageMaxDistanceFilter::~ImageMaxDistanceFilter()
  {
    if (maxDiff) delete [] maxDiff;
    if (lastImage) delete lastImage;
  }

  int
  ImageMaxDistanceFilter::getNumImagesAdded() const
  {
    return numImagesAdded;
  }

  void
  ImageMaxDistanceFilter::initialize(int w, int h)
  {
    if (maxDiff) delete [] maxDiff;
    if (lastImage) delete lastImage;
  
    maxDiff = new double[w*h];
    lastImage = 0;

    numImagesAdded = 0;  
    this->w = w;
    this->h = h;
  }

  void
  ImageMaxDistanceFilter::add(const Image& image)
  {
    if (image.getWidth() != w || image.getHeight() != h) {
      throw TribotsException(__FILE__ "Image format does not match.");
    }
    if (lastImage == 0) {          // remember first frame
      lastImage = image.clone();
      
    }
    else {                         // calculate pixelwise difference 
      YUVTuple yuv1, yuv2;         // between images t and t-1
      for (int x=0; x < w; x++) {  
	for (int y=0; y < h; y++) {
	  lastImage->getPixelYUV(x,y, &yuv1);
	  image.getPixelYUV(x,y, &yuv2);
	  maxDiff[x+y*w] = max(maxDiff[x+y*w], yuvDistanceSquare(yuv1, yuv2));
	}
      }
      numImagesAdded++;
    }
  }

  Image*
  ImageMaxDistanceFilter::createMask(double threshold)
  {
    if (numImagesAdded <= 0) {
      throw TribotsException(__FILE__ "Can not create mask, no images added.");
    }
    if (lastImage == 0) {        // assertion 
      throw TribotsException(__FILE__ "FATAL: lastImage==0, although "
			     "numImagesAdded > 0 !");
    }
    Image* image = new RGBImage(w,h);
    RGBTuple white = { 255, 255, 255 };
    
    double tmp;
    for (int x=0; x < w; x++) {
      for (int y=0; y < h; y++) {          // calculate mean
	tmp = sqrt(maxDiff[x+y*w]);
	if (tmp > threshold) {
	  image->setPixelRGB(x, y, white);
	}
      }
    }
    return image;
  }




}
