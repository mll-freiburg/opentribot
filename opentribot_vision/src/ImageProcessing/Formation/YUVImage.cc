#include "YUVImage.h"
#include "PixelConversion.h"
#include "ImageConversion.h"

namespace Tribots {

  Image* 
  YUVImage::clone() const
  {
    YUVImage* image = new YUVImage(buffer.width, buffer.height); 
                   // constructs new, empty buffer
    ImageBuffer::convert(buffer, image->getImageBuffer()); // memcopies
    return image;
  }

  YUVImage::YUVImage(const ImageBuffer& buffer) 
    : Image(), controlsBuffer(false)
  {
    if (buffer.format == ImageBuffer::FORMAT_YUV444) {
      this->buffer = buffer;
    }
    else {
      const ImageConverter* convert = 
	ImageConversionRegistry::getInstance()
	->getConverter(buffer.format, ImageBuffer::FORMAT_YUV444);
      // the preceeding call may have thrown an exception.

      this->buffer = 
	ImageBuffer(buffer.width, buffer.height, ImageBuffer::FORMAT_YUV444,
		    new unsigned char[buffer.width*buffer.height*3],
		    buffer.width* buffer.height * 3, buffer.timestamp);

      (*convert)(buffer, this->buffer);  // may throw an exception
      controlsBuffer = true;             // remember to delete the buffer
    }
  }

  YUVImage::YUVImage(int width, int height)
  {
    this->buffer = 
      ImageBuffer(width, height, ImageBuffer::FORMAT_YUV444,
		  new unsigned char[width*height*3],
		  width*height * 3);
    controlsBuffer = true;
  }

  YUVImage::~YUVImage()
  {
    if (controlsBuffer) {
      delete [] buffer.buffer;
    }
  }
  
  void
  YUVImage::getPixelYUV(int x, int y, YUVTuple* yuv) const
  {
    *yuv = reinterpret_cast<YUVTuple*>(buffer.buffer)[x+y*buffer.width];
  }					     
  
  void
  YUVImage::getPixelRGB(int x, int y, RGBTuple* rgb) const 
  {
    int y0, u, v, r, g, b;

    const YUVTuple& yuv = 
      reinterpret_cast<YUVTuple*>(buffer.buffer)[y*buffer.width+x];

    y0 = yuv.y;
    u  = yuv.u-128;
    v  = yuv.v-128;

    PIXEL_YUV2RGB(y0,u,v, r,g,b);
    
    rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
    rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
    rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));
  }

  void
  YUVImage::getPixelUYVY(int x, int y, UYVYTuple* uyvy) const 
  {
    int pos = y*buffer.width+x;
    const YUVTuple* yuv = 
      &reinterpret_cast<YUVTuple*>(buffer.buffer)[(pos/2)*2];

    uyvy->y1 = yuv->y;
    int u    = yuv->u/2;
    int v    = yuv->v/2;

    yuv++;

    uyvy->y2 = yuv->y;
    uyvy->u  = static_cast<unsigned char>((u+yuv->u)/2);
    uyvy->v  = static_cast<unsigned char>((v+yuv->v)/2);
  }

  unsigned char
  YUVImage::getPixelClass(int x, int y) const
  {
    return classifier->lookup(
      reinterpret_cast<YUVTuple*>(buffer.buffer)[x+y*buffer.width]);
  }

  void
  YUVImage::setPixelUYVY(int x, int y, const UYVYTuple& uyvy)
  {
    int pos = y*buffer.width+x;
    YUVTuple& yuv = reinterpret_cast<YUVTuple*>(buffer.buffer)[pos];

    if (pos % 2 == 0) {
      yuv.y = uyvy.y1;
    }
    else {
      yuv.y = uyvy.y2;
    }
    yuv.u = uyvy.u;
    yuv.v = uyvy.v;
  }

  void
  YUVImage::setPixelRGB(int x, int y, const RGBTuple& rgb)
  {
    int y0, u, v;

    YUVTuple& yuv = 
      reinterpret_cast<YUVTuple*>(buffer.buffer)[y*buffer.width+x];

    PIXEL_RGB2YUV(rgb.r, rgb.g, rgb.b, y0, u, v);
    
    yuv.y = static_cast<unsigned char>(y0<0 ? 0 : (y0>255 ? 255 : y0));
    yuv.u = static_cast<unsigned char>(u <0 ? 0 : (u >255 ? 255 : u ));
    yuv.v = static_cast<unsigned char>(v <0 ? 0 : (v >255 ? 255 : v ));
  }

  void
  YUVImage::setPixelYUV(int x, int y, const YUVTuple& yuv)
  {
    reinterpret_cast<YUVTuple*>(buffer.buffer)[(x+y*buffer.width)] = yuv;
  }

  void YUVImage::setBorder(const RGBTuple& rgb) 
  {
    YUVTuple yuv;

    PixelConversion::convert(rgb, &yuv);
    
    int w = getWidth()-1;
    int h = getHeight()-1;

    for (int x=w; x >= 0; x--) {
      setPixelRGB(x, 0, rgb);
      setPixelRGB(x, h, rgb);
    }
    for (int y=h; y >= 0; y--) {
      setPixelRGB(0, y, rgb);
      setPixelRGB(w, y, rgb);
    }
  }      


}
