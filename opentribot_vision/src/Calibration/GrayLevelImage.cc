
#include "GrayLevelImage.h"

using namespace Tribots;

GrayLevelImage::GrayLevelImage (unsigned int w, unsigned int h) :
    Table<float> (w,h) {;}

GrayLevelImage::GrayLevelImage (const GrayLevelImage& g) :
    Table<float> (g) {;}

GrayLevelImage::GrayLevelImage (const Image& img) :
    Table<float> (img.getWidth(), img.getHeight()) {
  YUVTuple yuv;
  for (int y=0; y<img.getHeight(); y++) {
    for (int x=0; x<img.getWidth(); x++) {
      img.getPixelYUV (x,y, &yuv);
      operator() (x,y) = static_cast<unsigned char> (yuv.y);
    }
  }
}

void GrayLevelImage::toImage (Image& dest, float offset, float scaling) const {
  YUVTuple yuv;
  yuv.u=yuv.v=127;
  for (unsigned int y=0; y<height(); y++) {
    for (unsigned int x=0; x<width(); x++) {
      float y1 = offset+scaling*operator() (x,y);
      yuv.y = static_cast<unsigned char>(y1<0 ? 0 : (y1>255 ? 255 : y1));
      dest.setPixelYUV (x,y, yuv);
    }
  }
}
