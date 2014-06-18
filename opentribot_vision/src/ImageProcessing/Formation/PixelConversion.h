
#ifndef _Tribots_PixelConversion_h_
#define _Tribots_PixelConversion_h_

#include "ColorTuples.h"

namespace Tribots {

  /**
   * Funktionen zur Umwandlung von einzelnen Pixeln zwischen den verschiedenen
   * Formaten. Die Bildklassen verwenden aus Geschwindigkeitsgründen zum
   * Teil eigene Umwandlungsfunktionen, die generell vorzuziehen sind.
   */
  class PixelConversion {
    public:
      static void convert(const RGBTuple&  rgb,  YUVTuple* yuv);
      static void convert(const YUVTuple&  yuv,  RGBTuple* rgb);
      static void convert(const UYVYTuple& uyvy, YUVTuple* yuv, int pos=0);
      static void convert(const UYVYTuple& uyvy, RGBTuple* rgb, int pos=0);
      static void convert(const YUV411Tuple& yuv411, YUVTuple* yuv, int pos=0);
      static void convert(const YUV411Tuple& yuv411, RGBTuple* rgb, int pos=0);
  };


#define PIXEL_YUV2RGB(y,u,v, r,g,b)                        \
      ((r) = (y)+(((v)*1436)>>10),                         \
       (g) = (y)-(((u)*352+(v)*731)>>10),                  \
       (b) = (y)+(((u)*1814)>>10)) 


#define PIXEL_RGB2YUV(r, g, b, y, u, v)                    \
      ((y) = (306*(r) + 601*(g) + 117*(b))  >> 10,         \
       (u) = ((-172*(r) - 340*(g) + 512*(b)) >> 10) + 128, \
       (v) = ((512*(r) - 429*(g) - 83*(b)) >> 10) + 128)

  // inline //////////////////////////////

  inline void 
      PixelConversion::convert(const RGBTuple&  rgb,  YUVTuple* yuv)
  {
    int y, u, v;
    PIXEL_RGB2YUV(rgb.r, rgb.g, rgb.b, y, u, v);
    yuv->y = static_cast<unsigned char>(y <0 ? 0 : (y >255 ? 255 : y ));
    yuv->u = static_cast<unsigned char>(u <0 ? 0 : (u >255 ? 255 : u ));
    yuv->v = static_cast<unsigned char>(v <0 ? 0 : (v >255 ? 255 : v ));
  }

  inline void 
      PixelConversion::convert(const YUVTuple&  yuv,  RGBTuple* rgb)
  {
    int y, u, v, r, g, b;

    y = yuv.y;
    u = yuv.u-128;
    v = yuv.v-128;

    PIXEL_YUV2RGB(y,u,v, r,g,b);
    
    rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
    rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
    rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));
  }

  inline void 
      PixelConversion::convert(const UYVYTuple& uyvy, YUVTuple* yuv, int pos) 
  {
    if (pos % 2 == 0) {
      yuv->y = uyvy.y1;
    }
    else {
      yuv->y = uyvy.y2;
    }
    yuv->u = uyvy.u;
    yuv->v = uyvy.v;
  }

  inline void 
      PixelConversion::convert(const UYVYTuple& uyvy, RGBTuple* rgb, int pos) 
  {
      int y, u, v, r, g, b;

      y = (pos%2 == 0) ? uyvy.y1 : uyvy.y2;
      u = uyvy.u-128;
      v = uyvy.v-128;

      PIXEL_YUV2RGB(y,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));
  }

  inline void
  PixelConversion::convert(const YUV411Tuple& yuv411, YUVTuple* yuv, int pos) {
    ++pos;
    if(pos>2) ++pos;

    yuv->y = reinterpret_cast<const unsigned char*>(&yuv411)[pos];
    yuv->u = yuv411.u;
    yuv->v = yuv411.v;
  }

  inline void
  PixelConversion::convert(const YUV411Tuple& yuv411, RGBTuple* rgb, int pos) {
    int y, u, v, r, g, b;
    ++pos;
    if(pos>2) ++pos;

    y = (reinterpret_cast<const unsigned char*>(&yuv411)[pos]);
    u = yuv411.u-128;
    v = yuv411.v-128;

    PIXEL_YUV2RGB(y,u,v, r,g,b);

    rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
    rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
    rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));
  }
}

#endif
