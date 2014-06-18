
#ifndef _gradient_h
#define _gradient_h

#include "Raster.h"
#include "PGMFrame.h"

struct PolarGradient {
  unsigned char sector;    // 0=nach rechts ... 2=nach oben ... 4=nach links ... 6=nach unten ...
  unsigned char magnitude; // 0 ... 254
};


Raster<PolarGradient>* sobel_gradient (const PGMFrame&); // Gradient berechnen mit Sobel


#endif



