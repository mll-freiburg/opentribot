
#ifndef _TribotsTools_ImageMaskBuilder_h_
#define _TribotsTools_ImageMaskBuilder_h_

#include "../../ImageProcessing/Formation/Image.h"
#include "../../ImageProcessing/PixelAnalysis/RobotMask.h"
#include <vector>

namespace TribotsTools {

  class ImageMaskBuilder {
  public:
    ImageMaskBuilder ();
    ~ImageMaskBuilder ();

    void init (unsigned int width1, unsigned int height1);
    void addImage (const Tribots::Image&);
    Tribots::RobotMask* generateMask (double threshold =10);
    Tribots::RobotMask* dilateMask (const Tribots::RobotMask* mask, unsigned int numDilate =6);
    unsigned int numSamples () const throw ();

  private:
    unsigned int width;
    unsigned int height;
    std::vector<double> g;   ///< zum Berechnen von SUM_Images_i (g_i (x,y))
    std::vector<double> gg;  ///< zum Berechnen von SUM_Images_i (g_i (x,y)^2)
    unsigned int num;
  };

}

#endif
