
#ifndef _Tribots_ColorTuples_h_
#define _Tribots_ColorTuples_h_

namespace Tribots {

  /** Holds a single RGB color value. r,g and b in [0,...,255]. */
  typedef struct {
    unsigned char r; ///< red value in [0,...,255]
    unsigned char g; ///< green value in [0,...,255]
    unsigned char b; ///< blue value in [0,...,255]
  } RGBTuple;

  /** Holds a single YUV color value.
   *  y in [0,...,255], u and v in [0,...,255]. */
  typedef struct {
    unsigned char y; ///< luminance in [0,...,255]
    unsigned char u; ///< u in [0,...,255]
    unsigned char v; ///< v in [0,...,255]
  } YUVTuple;

  /** Holds a single YUV color value ordered u y v.
   *  y in [0,...,255], u and v in [0,...,255]. */
  typedef struct {
    unsigned char u; ///< u in [0,...,255]
    unsigned char y; ///< luminance in [0,...,255]
    unsigned char v; ///< v in [0,...,255]
  } UYVTuple;

  /** Holds two YUV color values (4:2:2). 
   *  y1 and y2 in [0,...,255], u and v in [0,...,255]. */
  typedef struct {
    unsigned char u; ///< u in [0,...,255]
    unsigned char y1;///< luminance of first pixel in [0,...,255]
    unsigned char v; ///< v in [0,...,255]
    unsigned char y2;///< luminance of second pixel in [0,...,255]
  } UYVYTuple;

  /** YUV (4:1:1) format.
   */
  typedef struct {
    unsigned char u;  ///< u in [0,...,255]
    unsigned char y1; ///< luminance of first pixel in [0,...,255]
    unsigned char y2; ///< luminance of second pixel in [0,...,255]
    unsigned char v;  ///< v in [0,...,255]
    unsigned char y3; ///< luminance of third pixel in [0,...,255]
    unsigned char y4; ///< luminance of fouth pixel in [0,...,255]

  } YUV411Tuple;

}

inline bool operator== (const Tribots::RGBTuple& r1, const Tribots::RGBTuple& r2) {
  return (r1.r==r2.r && r1.g==r2.g && r1.b==r2.b);
}

inline bool operator!= (const Tribots::RGBTuple& r1, const Tribots::RGBTuple& r2) {
  return (r1.r!=r2.r || r1.g!=r2.g || r1.b!=r2.b);
}

#endif
