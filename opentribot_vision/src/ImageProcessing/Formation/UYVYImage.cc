#include "UYVYImage.h"
#include "PixelConversion.h"
#include "ImageConversion.h"

namespace Tribots {

  Image* 
  UYVYImage::clone() const
  {
    UYVYImage* image = new UYVYImage(buffer.width, buffer.height); 
                   // constructs new, empty buffer
    ImageBuffer::convert(buffer, image->getImageBuffer()); // memcopies
    return image;
  }

  UYVYImage::UYVYImage(const ImageBuffer& buffer) 
    : Image(), controlsBuffer(false)
  {
    if (buffer.format == ImageBuffer::FORMAT_YUV422) {
      this->buffer = buffer;
    }
    else {
      const ImageConverter* convert = 
	ImageConversionRegistry::getInstance()
	->getConverter(buffer.format, ImageBuffer::FORMAT_YUV422);
      // the preceeding call may have thrown an exception.

      this->buffer = 
	ImageBuffer(buffer.width, buffer.height, ImageBuffer::FORMAT_YUV422,
		    new unsigned char[buffer.width*buffer.height*2],
		    buffer.width* buffer.height * 2, buffer.timestamp);

      (*convert)(buffer, this->buffer);  // may throw an exception
      controlsBuffer = true;             // remember to delete the buffer
    }
  }

  UYVYImage::UYVYImage(int width, int height)
  {
    this->buffer = 
      ImageBuffer(width, height, ImageBuffer::FORMAT_YUV422,
		  new unsigned char[width*height*2],
		  width*height * 2);
    controlsBuffer = true;
  }

  UYVYImage::~UYVYImage()
  {
    if (controlsBuffer) {
      delete [] buffer.buffer;
    }
  }
  
  void
  UYVYImage::getPixelYUV(int x, int y, YUVTuple* yuv) const
  {
    int pos = y*buffer.width+x;
    const UYVYTuple& uyvy = reinterpret_cast<UYVYTuple*>(buffer.buffer)[pos/2];

    yuv->y = pos % 2 == 0 ? uyvy.y1 : uyvy.y2;
    yuv->u = uyvy.u;
    yuv->v = uyvy.v;
  }					     
  
  void
  UYVYImage::getPixelRGB(int x, int y, RGBTuple* rgb) const 
  {
    int pos = y*buffer.width+x;
    int y0, u, v, r, g, b;

    const UYVYTuple& uyvy = 
      reinterpret_cast<UYVYTuple*>(buffer.buffer)[pos/2];

    y0 = x%2==0 ? uyvy.y1 : uyvy.y2;  // assumes y%2 == 0
    u = uyvy.u-128;
    v = uyvy.v-128;

    PIXEL_YUV2RGB(y0,u,v, r,g,b);
    
    rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
    rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
    rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));
  }

  void
  UYVYImage::getPixelUYVY(int x, int y, UYVYTuple* uyvy) const 
  {
    *uyvy = reinterpret_cast<UYVYTuple*>(buffer.buffer)[(x+y*buffer.width)/2];
  }

  unsigned char
  UYVYImage::getPixelClass(int x, int y) const
  {
    int pos = x+y*buffer.width;
    return classifier->lookup(
      reinterpret_cast<UYVYTuple*>(buffer.buffer)[pos/2], pos%2);
  }


  void
  UYVYImage::setPixelYUV(int x, int y, const YUVTuple& yuv)
  {
    int pos = y*buffer.width+x;
    UYVYTuple& uyvy = reinterpret_cast<UYVYTuple*>(buffer.buffer)[pos/2];

    if (pos % 2 == 0) {
      uyvy.y1 = yuv.y;
    }
    else {
      uyvy.y2 = yuv.y;
    }
    uyvy.u = yuv.u;
    uyvy.v = yuv.v;
  }

  void
  UYVYImage::setPixelRGB(int x, int y, const RGBTuple& rgb)
  {
    int pos = y*buffer.width+x;
    int y0, u, v, r, g, b;

    UYVYTuple& uyvy = 
      reinterpret_cast<UYVYTuple*>(buffer.buffer)[pos/2];

    r = rgb.r;
    g = rgb.g;
    b = rgb.b;

    PIXEL_RGB2YUV(r, g, b, y0, u, v);
    
    if (pos%2 == 0) {
      uyvy.y1 = static_cast<unsigned char>(y0<0 ? 0 : (y0>255 ? 255 : y0));
    }
    else {
      uyvy.y2 = static_cast<unsigned char>(y0<0 ? 0 : (y0>255 ? 255 : y0));
    }
    uyvy.u = static_cast<unsigned char>(u<0 ? 0 : (u>255 ? 255 : u));
    uyvy.v = static_cast<unsigned char>(v<0 ? 0 : (v>255 ? 255 : v));
  }

  void
  UYVYImage::setPixelUYVY(int x, int y, const UYVYTuple& uyvy)
  {
    reinterpret_cast<UYVYTuple*>(buffer.buffer)[(x+y*buffer.width)/2] = uyvy;
  }

}
