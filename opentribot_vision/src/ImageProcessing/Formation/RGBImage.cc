#include "RGBImage.h"
#include "PixelConversion.h"
#include "ImageConversion.h"

namespace Tribots {

  Image* RGBImage::clone() const
  {
    RGBImage* image = new RGBImage(buffer.width, buffer.height); 
                   // constructs new, empty buffer
    ImageBuffer::convert(buffer, image->getImageBuffer()); // memcopies
    return image;
  }

  RGBImage::RGBImage(const ImageBuffer& buffer) 
    : Image(), controlsBuffer(false)
  {
    if (buffer.format == ImageBuffer::FORMAT_RGB) {
      this->buffer = buffer;
    }
    else {
      const ImageConverter* convert = 
	ImageConversionRegistry::getInstance()
	->getConverter(buffer.format, ImageBuffer::FORMAT_RGB);
      // the last call may have thrown an exception.

      this->buffer = 
	ImageBuffer(buffer.width, buffer.height, ImageBuffer::FORMAT_RGB,
		    new unsigned char[buffer.width*buffer.height*3],
		    buffer.width* buffer.height * 3, buffer.timestamp);

      (*convert)(buffer, this->buffer);  // may throw an exception
      controlsBuffer = true;             // remember to delete the buffer
    }
  }

  RGBImage::RGBImage(int width, int height)
  {
    this->buffer = 
      ImageBuffer(width, height, ImageBuffer::FORMAT_RGB,
		  new unsigned char[width*height*3],
		  width*height * 3);
    controlsBuffer = true;
  }

  RGBImage::~RGBImage()
  {
    if (controlsBuffer) {
      delete [] buffer.buffer;
    }
  }
  
  void  
  RGBImage::getPixelYUV(int x, int y, YUVTuple* yuv) const
  {
    int y0, u, v;

    const RGBTuple& rgb = 
      reinterpret_cast<RGBTuple*>(buffer.buffer)[x+y*buffer.width];

    PIXEL_RGB2YUV(rgb.r,rgb.g,rgb.b, y0,u,v);
    
    yuv->y = static_cast<unsigned char>(y0<0 ? 0 : (y0>255 ? 255 : y0));
    yuv->u = static_cast<unsigned char>(u <0 ? 0 : (u >255 ? 255 : u ));
    yuv->v = static_cast<unsigned char>(v <0 ? 0 : (v >255 ? 255 : v ));
  }					     

  void 
  RGBImage::getPixelUYVY(int x, int y, UYVYTuple* uyvy) const
  {
    int y1,y2, u1, v1, u2, v2;

    const RGBTuple* rgb =  
      &reinterpret_cast<RGBTuple*>(buffer.buffer)[(x+y*buffer.width)/2*2];
                  // point to even(!) entry

    PIXEL_RGB2YUV(rgb->r,rgb->g,rgb->b, y1,u1,v1);
    ++rgb;
    PIXEL_RGB2YUV(rgb->r,rgb->g,rgb->b, y2,u2,v2);

    u1 = (u1+u2) / 2;
    v1 = (v1+v2) / 2;
    
    uyvy->y1 = static_cast<unsigned char>(y1<0 ? 0 : (y1>255 ? 255 : y1));
    uyvy->y2 = static_cast<unsigned char>(y2<0 ? 0 : (y2>255 ? 255 : y2));
    uyvy->u  = static_cast<unsigned char>(u1<0 ? 0 : (u1>255 ? 255 : u1));
    uyvy->v  = static_cast<unsigned char>(v1<0 ? 0 : (v1>255 ? 255 : v1));   
  }

  void 
  RGBImage::getPixelRGB(int x, int y, RGBTuple* rgb) const 
  {
    *rgb = reinterpret_cast<RGBTuple*>(buffer.buffer)[x+y*buffer.width];
  }

  unsigned char
  RGBImage::getPixelClass(int x, int y) const
  {
    return classifier->lookup(
      reinterpret_cast<RGBTuple*>(buffer.buffer)[x+y*buffer.width]);
  }


  void
  RGBImage::setPixelYUV(int x, int y, const YUVTuple& yuv)
  {
    RGBTuple& rgb = 
      reinterpret_cast<RGBTuple*>(buffer.buffer)[x+buffer.width*y];

    int y0,u,v, r,g,b;
    y0=yuv.y;
    u =yuv.u-128;
    v =yuv.v-128;

    PIXEL_YUV2RGB(y0,u,v, r,g,b);

    rgb.r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
    rgb.g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
    rgb.b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));
  }

  void
  RGBImage::setPixelRGB(int x, int y, const RGBTuple& rgb)
  {
    reinterpret_cast<RGBTuple*>(buffer.buffer)[x+buffer.width*y] = rgb;
  }

  void
  RGBImage::setPixelUYVY(int x, int y, const UYVYTuple& uyvy)
  {
    int pos = y*buffer.width+x;
    int y0, u, v, r, g, b;

    RGBTuple& rgb = 
      reinterpret_cast<RGBTuple*>(buffer.buffer)[pos];

    y0 = (pos%2 == 0) ? uyvy.y1 : uyvy.y2;
    u  = uyvy.u-128;
    v  = uyvy.v-128;

    PIXEL_YUV2RGB(y0,u,v, r,g,b);

    rgb.r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
    rgb.g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
    rgb.b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));
  }


}
