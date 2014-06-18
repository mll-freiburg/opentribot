#include "YUV411Image.h"
#include "ImageConversion.h"
#include "../PixelAnalysis/ColorClassifier.h"
#include "PixelConversion.h"

namespace Tribots {

  Image* 
  YUV411Image::clone() const
  {
    YUV411Image* image = new YUV411Image(buffer.width, buffer.height); 
                   // constructs new, empty buffer
    ImageBuffer::convert(buffer, image->getImageBuffer()); // memcopies
    return image;
  }

  YUV411Image::YUV411Image(const ImageBuffer& buffer) 
    : Image(), controlsBuffer(false)
  {
    if (buffer.format == ImageBuffer::FORMAT_YUV411) {
      this->buffer = buffer;
    }
    else {
    
      const ImageConverter* convert = 
	ImageConversionRegistry::getInstance()
	->getConverter(buffer.format, ImageBuffer::FORMAT_YUV411);
      // the preceeding call may have thrown an exception.

      this->buffer = 
	ImageBuffer(buffer.width, buffer.height, ImageBuffer::FORMAT_YUV411,
		    new unsigned char[(int)(buffer.width*buffer.height*1.5)],
		    (unsigned int)(buffer.width* buffer.height * 1.5), buffer.timestamp);

      (*convert)(buffer, this->buffer);  // may throw an exception
      controlsBuffer = true;             // remember to delete the buffer
    
    }
  }

  YUV411Image::YUV411Image(int width, int height)
  {
    this->buffer = 
      ImageBuffer(width, height, ImageBuffer::FORMAT_YUV411,
		  new unsigned char[(unsigned int)(width*height*1.5)],
		  (int)(width*height * 1.5));
    controlsBuffer = true;
  }

  YUV411Image::~YUV411Image()
  {
    if (controlsBuffer) {
      delete [] buffer.buffer;
    }
  }
  
  void
  YUV411Image::getPixelYUV(int x, int y, YUVTuple* yuv) const
  {
    int pos = y*buffer.width+x;
    const YUV411Tuple& yuv411 = reinterpret_cast<YUV411Tuple*>(buffer.buffer)[pos/4];

    PixelConversion::convert(yuv411, yuv);
    
  }					     

  void
  YUV411Image::getPixelUYVY(int x, int y, UYVYTuple* uyuv) const
  {
    int pos = y*buffer.width+x;
    const YUV411Tuple& yuv411 = reinterpret_cast<YUV411Tuple*>(buffer.buffer)[pos/4];
  
    if(pos%4>1){
      uyuv->y1 = yuv411.y3;
      uyuv->y2 = yuv411.y4;
    }else{
      uyuv->y1 = yuv411.y1;
      uyuv->y2 = yuv411.y2;
    }
    uyuv->u = yuv411.u;
    uyuv->v = yuv411.v;
    
  }     

    
  void
  YUV411Image::getPixelRGB(int x, int y, RGBTuple* rgb) const 
  {
    int pos = y*buffer.width+x;
    const YUV411Tuple& yuv411 = reinterpret_cast<YUV411Tuple*>(buffer.buffer)[pos/4];
    PixelConversion::convert(yuv411, rgb);
  }

  unsigned char
  YUV411Image::getPixelClass(int x, int y) const
  {
    // TODO: optimize
    YUVTuple yuv;
    getPixelYUV(x,y,&yuv);
    
    return classifier->lookup(yuv);

  }


  void
  YUV411Image::setPixelYUV(int x, int y, const YUVTuple& yuv)
  {
    int pos = y*buffer.width+x;

    YUV411Tuple& yuv411 = 
      reinterpret_cast<YUV411Tuple*>(buffer.buffer)[pos/4];

    pos %= 4;    
    ++pos;
    if(pos>2) ++pos;  
    
    reinterpret_cast<unsigned char*>(&yuv411)[pos] = yuv.y;
       
    yuv411.u = yuv.u;
    yuv411.v = yuv.v;
  }

  void
  YUV411Image::setPixelRGB(int x, int y, const RGBTuple& rgb)
  {
    
    int y0, u, v, r, g, b;
    int pos = y*buffer.width+x;

    YUV411Tuple& yuv411 = 
      reinterpret_cast<YUV411Tuple*>(buffer.buffer)[pos/4];

    r = rgb.r;
    g = rgb.g;
    b = rgb.b;

    PIXEL_RGB2YUV(r, g, b, y0, u, v);

    pos %= 4;    
    
    ++pos;
    if(pos>2) ++pos;  
    
    reinterpret_cast<unsigned char*>(&yuv411)[pos]
       = static_cast<unsigned char>(y0<0 ? 0 : (y0>255 ? 255 : y0));
    yuv411.u = static_cast<unsigned char>(u<0 ? 0 : (u>255 ? 255 : u));
    yuv411.v = static_cast<unsigned char>(v<0 ? 0 : (v>255 ? 255 : v));
    
  }

  void
  YUV411Image::setPixelUYVY(int x, int y, const UYVYTuple& uyvy)
  {
    int pos = y*buffer.width+x;
    YUV411Tuple& yuv411 = reinterpret_cast<YUV411Tuple*>(buffer.buffer)[pos/4];
  
    if(pos%4>1){
      yuv411.y3 = uyvy.y1;
      yuv411.y4 = uyvy.y2;
    }else{
      yuv411.y1 = uyvy.y1;
      yuv411.y2 = uyvy.y2;
    }
    yuv411.u = uyvy.u;
    yuv411.v = uyvy.v;
    
  }     
}
