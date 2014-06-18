
#ifndef _Tribots_Convolution_h_
#define _Tribots_Convolution_h_

#include "GrayLevelImage.h"
#include <vector>

namespace Tribots {

  /** generic convolution operator. Derive child classes for Gaussian
      smoothing, Sobel, etc. It might happen that the boundary area
      of the resulting image is invalid (size of the boundary is given by
      boundarySizeX(), boundarySizeY()).
      TODO: more efficient implementation */
  class Convolution {
  protected:
    int dimX;  ///< horizontal mask size
    int dimY;  ///< vertical mask size
    std::vector<float> mask;  ///< mask parameters row by row
    float offset;  ///< offset

    /** override in child class */
    Convolution () throw ();
  public:
    /** execute convolution with mask, src is the src image, dest is the
        destination, both images must be of the same size */
    void operator () (GrayLevelImage& dest, const GrayLevelImage& src) throw ();
    /** get the number of invalid pixels on the horizontal boundary */
    unsigned int boundarySizeX() const throw ();
    /** get the number of invalid pixels on the vertical boundary */
    unsigned int boundarySizeY() const throw ();
  };

  /** Gaussian smoothing operator, provide the mask width (3,5,7,...)
      in the constructor */
  class GaussianSmoothing : public Convolution {
  public:
    /** only use width = 3, 5, 7, ... */
    GaussianSmoothing (unsigned int width) throw ();
  };

  /** Sobel operator with 3x3 mask for dI/dx */
  class SobelX : public Convolution {
  public:
    SobelX () throw ();
  };

  /** Sobel operator with 3x3 mask for dI/dy */
  class SobelY : public Convolution {
  public:
    SobelY () throw ();
  };

}

#endif
