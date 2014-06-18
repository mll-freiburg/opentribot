
#ifndef _Tribots_Pixelsets_h_
#define _Tribots_Pixelsets_h_

#include <deque>
#include "ImageCoordinate.h"
#include "../Formation/Image.h"

namespace Tribots {

  typedef std::deque<ImageCoordinate> Pixelset;

  /** set pixelset to all pixels of an image of given width and height */
  void setPixelsetAll (Pixelset& dest, unsigned int width, unsigned int height) throw (std::bad_alloc);

  /** find all bluish pixels in src and return a sorted list in dest */
  void findBluishPixels (Pixelset& dest, const Image& src, double sensitivity=1.2) throw (std::bad_alloc);

  /** find all greenish pixels in src and return a sorted list in dest */
  void findGreenishPixels (Pixelset& dest, const Image& src, double sensitivity=1.2) throw (std::bad_alloc);

  /** find all reddish pixels in src and return a sorted list in dest */
  void findReddishPixels (Pixelset& dest, const Image& src, double sensitivity=1.2) throw (std::bad_alloc);

  /** intersect sorted pixelsets */
  void intersectSortedSets (Pixelset& dest, const Pixelset& src1, const Pixelset& src2) throw (std::bad_alloc);

  /** calculate the center of gravity */
  Vec centerOfGravity (const Pixelset& src) throw();

  /** in image 'image' set color of allpixels in 'pixelset' to color 'color'.
      No range check is done. */
  void drawPixelset (Image& image, const Pixelset& pixelset, RGBTuple color) throw ();
}

#endif
