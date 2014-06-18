
#include "gradient.h"
#include <cmath>

Raster<PolarGradient>* sobel_gradient (const PGMFrame& src) {
  unsigned int width = src.get_width()-2;
  unsigned int height = src.get_height()-2;
  Raster<int> zeilenableitung (width, height+2);
  Raster<int> spaltenableitung (width+2, height);
  for (unsigned int y=0; y<height+2; y++)
    for (unsigned int x=0; x<width; x++) {
      zeilenableitung (x,y) = src.get_grey(x+2,y)-src.get_grey(x,y);
    }
  for (unsigned int y=0; y<height; y++)
    for (unsigned int x=0; x<width+2; x++) {
      spaltenableitung (x,y) = src.get_grey(x,y)-src.get_grey(x,y+2);
    }
  Raster<PolarGradient>* grad = new Raster<PolarGradient> (width, height);
  for (unsigned int y=0; y<height; y++)
    for (unsigned int x=0; x<width; x++) {
      float dgdx = zeilenableitung (x,y)+2*zeilenableitung (x,y+1)+zeilenableitung (x,y+2);
      float dgdy = spaltenableitung (x,y)+2*spaltenableitung (x+1,y)+spaltenableitung (x+2,y);
      float dgdxrot = 0.92388f*dgdx-0.382683f*dgdy;
      float dgdyrot = 0.382683f*dgdx+0.92388f*dgdy;
      if (dgdxrot>=0) 
	if (dgdyrot>=0)
	  (*grad)(x,y).sector = dgdxrot>=dgdyrot ? 1 : 2;
	else
	  (*grad)(x,y).sector = dgdxrot>=-dgdyrot ? 8 : 7;
      else
	if (dgdyrot>=0)
	  (*grad)(x,y).sector = -dgdxrot>=dgdyrot ? 4 : 3;
	else
	  (*grad)(x,y).sector = dgdxrot<=dgdyrot ? 5 : 6;
      (*grad)(x,y).magnitude = static_cast<unsigned char>(sqrt(dgdx*dgdx+dgdy*dgdy)*0.235702f);
    }
  return grad;
}



