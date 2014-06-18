#include "ImageConversion.h"
#include "PixelConversion.h"
#include <sstream>
#include "cstring"

namespace Tribots {



  ImageConverter::ImageConverter(int srcFormat, int dstFormat)
    : srcFormat(srcFormat), dstFormat(dstFormat)
  {}

  CopyConverter::CopyConverter() 
    : ImageConverter(-1, -1)
  {}

  void
  CopyConverter::operator() (const ImageBuffer& src, ImageBuffer& dst) const
    throw (TribotsException)
  {
    if (src.format != dst.format) {
      throw TribotsException(__FILE__
			     ": CopyConverter: Source and target buffers do "
			     " not have the same format");
    }

    memcpy(dst.buffer, src.buffer, dst.size);
  }


  YUV2RGB::YUV2RGB() : ImageConverter(ImageBuffer::FORMAT_YUV444,
				      ImageBuffer::FORMAT_RGB)
  {}

  void
  YUV2RGB::operator() (const ImageBuffer& src, ImageBuffer& dst) const
    throw (TribotsException)
  {
    if (src.format != ImageBuffer::FORMAT_YUV444 ||
	dst.format != ImageBuffer::FORMAT_RGB) {
      throw TribotsException(__FILE__
			     ": YUV2RGB: Source and target buffers do not "
			     "have the expected format");
    }
    register YUVTuple* yuv = 
      reinterpret_cast<YUVTuple*>(src.buffer);
    register RGBTuple* rgb =
      reinterpret_cast<RGBTuple*>(dst.buffer);

    register int y,u,v, r,g,b;

    for(register int i = src.width * src.height; i > 0;	i--) {
      y = yuv->y;
      u = yuv->u-128;
      v = yuv->v-128;

      PIXEL_YUV2RGB(y,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));

      ++yuv; ++rgb;
    }
  }

  RGB2YUV::RGB2YUV() : ImageConverter(ImageBuffer::FORMAT_RGB,
				      ImageBuffer::FORMAT_YUV444)
  {}

  void
  RGB2YUV::operator() (const ImageBuffer& src, ImageBuffer& dst) const
    throw (TribotsException)
  {
    if (src.format != ImageBuffer::FORMAT_RGB ||
	dst.format != ImageBuffer::FORMAT_YUV444) {
      throw TribotsException(__FILE__
			     ": RGB2YUV: Source and target buffers do not "
			     "have the expected format");
    }
    register RGBTuple* rgb =
      reinterpret_cast<RGBTuple*>(src.buffer);
    register YUVTuple* yuv = 
      reinterpret_cast<YUVTuple*>(dst.buffer);

    register int y,u,v;

    for(register int i = src.width * src.height; i > 0;	i--) {

      PIXEL_RGB2YUV(rgb->r, rgb->g, rgb->b, y, u, v);

      yuv->y = static_cast<unsigned char>(y <0 ? 0 : (y >255 ? 255 : y ));
      yuv->u = static_cast<unsigned char>(u <0 ? 0 : (u >255 ? 255 : u ));
      yuv->v = static_cast<unsigned char>(v <0 ? 0 : (v >255 ? 255 : v ));

      ++rgb; ++yuv;
    }
  }

  UYV2RGB::UYV2RGB() : ImageConverter(ImageBuffer::FORMAT_UYV,
				      ImageBuffer::FORMAT_RGB)
  {}

  void
  UYV2RGB::operator() (const ImageBuffer& src, ImageBuffer& dst) const
    throw (TribotsException)
  {
    if (src.format != ImageBuffer::FORMAT_UYV ||
	dst.format != ImageBuffer::FORMAT_RGB) {
      throw TribotsException(__FILE__
			     ": UYV2RGB: Source and target buffers do not "
			     "have the expected format");
    }
    register UYVTuple* uyv = 
      reinterpret_cast<UYVTuple*>(src.buffer);
    register RGBTuple* rgb =
      reinterpret_cast<RGBTuple*>(dst.buffer);

    register int y,u,v, r,g,b;

    for(register int i = src.width * src.height; i > 0;	i--) {
      y = uyv->y;
      u = uyv->u-128;
      v = uyv->v-128;

      PIXEL_YUV2RGB(y,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));

      ++uyv; ++rgb;
    }
  }

  UYVY2YUV::UYVY2YUV() : ImageConverter(ImageBuffer::FORMAT_YUV422,
					ImageBuffer::FORMAT_YUV444)
  {}

  void
  UYVY2YUV::operator() (const ImageBuffer& src, ImageBuffer& dst) const
    throw (TribotsException)
  {
    if (src.format != ImageBuffer::FORMAT_YUV422 ||
	dst.format != ImageBuffer::FORMAT_YUV444) {
      throw TribotsException(__FILE__
			     ": UYVY2YUV: Source and target buffers do not "
			     "have the expected format");
    }
    register UYVYTuple* uyvy = 
      reinterpret_cast<UYVYTuple*>(src.buffer);
    register YUVTuple* yuv =
      reinterpret_cast<YUVTuple*>(dst.buffer);

    for(register int i = src.width * src.height / 2; i > 0; i--) {
      
      yuv->y = uyvy->y1;
      yuv->u = uyvy->u;
      yuv->v = uyvy->v;

      ++yuv;

      yuv->y = uyvy->y2;
      yuv->u = uyvy->u;
      yuv->v = uyvy->v;

      ++uyvy; ++yuv;
    }
  }

  UYVY2RGB::UYVY2RGB() : ImageConverter(ImageBuffer::FORMAT_YUV422,
					ImageBuffer::FORMAT_RGB)
  {}

  void
  UYVY2RGB::operator() (const ImageBuffer& src, ImageBuffer& dst) const
    throw (TribotsException)
  {
    if (src.format != ImageBuffer::FORMAT_YUV422 ||
	dst.format != ImageBuffer::FORMAT_RGB) {
      throw TribotsException(__FILE__
			     ": UYVY2RGB: Source and target buffers do not "
			     "have the expected format");
    }
    register UYVYTuple* uyvy = 
      reinterpret_cast<UYVYTuple*>(src.buffer);
    register RGBTuple* rgb =
      reinterpret_cast<RGBTuple*>(dst.buffer);

    register int y1,y2,u,v, r,g,b;

    for(register int i = src.width * src.height / 2; i > 0; i--) {
      y1 = uyvy->y1;
      y2 = uyvy->y2;
      u  = uyvy->u-128;
      v  = uyvy->v-128;

      PIXEL_YUV2RGB(y1,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));

      ++rgb;

      PIXEL_YUV2RGB(y2,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));

      ++uyvy; ++rgb;
    }
  }


// ---- YUV411 2 YUV, YUV411 2 RGB ---------------------------------------------
  
  YUV4112YUV::YUV4112YUV() : ImageConverter(ImageBuffer::FORMAT_YUV411,
					    ImageBuffer::FORMAT_YUV444)
  {}

  void
  YUV4112YUV::operator() (const ImageBuffer& src, ImageBuffer& dst) const
    throw (TribotsException)
  {
    if (src.format != ImageBuffer::FORMAT_YUV411 ||
	dst.format != ImageBuffer::FORMAT_YUV444) {
      throw TribotsException(__FILE__
			     ": YUV4112YUV: Source and target buffers do not "
			     "have the expected format");
    }
    register YUV411Tuple* yuv411 = 
      reinterpret_cast<YUV411Tuple*>(src.buffer);
    register YUVTuple* yuv =
      reinterpret_cast<YUVTuple*>(dst.buffer);

    for(register int i = src.width * src.height / 4; i > 0; i--) {
      
      yuv->y = yuv411->y1;
      yuv->u = yuv411->u;
      yuv->v = yuv411->v;

      ++yuv;

      yuv->y = yuv411->y2;
      yuv->u = yuv411->u;
      yuv->v = yuv411->v;

      ++yuv;
      
      yuv->y = yuv411->y3;
      yuv->u = yuv411->u;
      yuv->v = yuv411->v;

      ++yuv;

      yuv->y = yuv411->y4;
      yuv->u = yuv411->u;
      yuv->v = yuv411->v;

      ++yuv; ++yuv411; 
      
    }
  }

  YUV4112RGB::YUV4112RGB() : ImageConverter(ImageBuffer::FORMAT_YUV411,
					ImageBuffer::FORMAT_RGB)
  {}

  void
  YUV4112RGB::operator() (const ImageBuffer& src, ImageBuffer& dst) const
    throw (TribotsException)
  {
    if (src.format != ImageBuffer::FORMAT_YUV411 ||
	dst.format != ImageBuffer::FORMAT_RGB) {
      throw TribotsException(__FILE__
			     ": YUV4112RGB: Source and target buffers do not "
			     "have the expected format");
    }
    register YUV411Tuple* yuv411 = 
      reinterpret_cast<YUV411Tuple*>(src.buffer);
    register RGBTuple* rgb =
      reinterpret_cast<RGBTuple*>(dst.buffer);

    register int y1,y2,y3,y4,u,v, r,g,b;

    for(register int i = src.width * src.height / 2; i > 0; i--) {
      y1 = yuv411->y1;
      y2 = yuv411->y2;
      y3 = yuv411->y3;
      y4 = yuv411->y4;
      u  = yuv411->u-128;
      v  = yuv411->v-128;

      PIXEL_YUV2RGB(y1,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));

      ++rgb;

      PIXEL_YUV2RGB(y2,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));

      ++rgb;
      
      PIXEL_YUV2RGB(y3,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));

      ++rgb;

      PIXEL_YUV2RGB(y4,u,v, r,g,b);

      rgb->r = static_cast<unsigned char>(r<0 ? 0 : (r>255 ? 255 : r));
      rgb->g = static_cast<unsigned char>(g<0 ? 0 : (g>255 ? 255 : g));
      rgb->b = static_cast<unsigned char>(b<0 ? 0 : (b>255 ? 255 : b));

      ++rgb; ++yuv411; 
    }
  }

  ImageConversionRegistry* 
  ImageConversionRegistry::singleton = 0;

  ImageConversionRegistry* 
  ImageConversionRegistry::getInstance()
  {
    if (singleton == 0) {
      singleton = new ImageConversionRegistry();
    }
    return singleton;
  }
  
  ImageConversionRegistry::ImageConversionRegistry()
  {
    copy = new CopyConverter();

    // you may add your implementation here or use registerConverter...

    registerConverter(new YUV2RGB());
    registerConverter(new RGB2YUV());
    registerConverter(new UYV2RGB());
    registerConverter(new UYVY2YUV());
    registerConverter(new UYVY2RGB());
    registerConverter(new YUV4112YUV());
    registerConverter(new YUV4112RGB());
  }

  ImageConversionRegistry::~ImageConversionRegistry()
  {
    delete copy;
    for (std::map<int, std::map<int, ImageConverter*> >::iterator it = 
	   converterMap.begin(); 
	 it != converterMap.end(); 
	 it++) {
      
      for (std::map<int, ImageConverter*>::iterator innerIt = 
	     it->second.begin(); 
	   innerIt != it->second.end(); 
	   innerIt++) {

	delete innerIt->second;       // delete ImageConverter object
      }
      it->second.clear();             // empty the inner map(s)
    }
    converterMap.clear();             // empty the outer map
  }

  void
  ImageConversionRegistry::registerConverter(ImageConverter* converter)
  {
    converterMap[converter->getSourceFormat()]
      [converter->getDestinationFormat()] = converter;
  }

  const ImageConverter* 
  ImageConversionRegistry::getConverter(int srcFormat, int dstFormat) 
    throw(TribotsException)
  {
    if (srcFormat == dstFormat) {
      return copy;
    }

    if (converterMap[srcFormat].find(dstFormat) == 
	converterMap[srcFormat].end()) {
      std::stringstream iostream;
      iostream << __FILE__ << ": The requested image converter from " << srcFormat << " to " << dstFormat << " is not implemented.\n";
      std::string errmsg;
      std::getline (iostream, errmsg);
      throw TribotsException(errmsg.c_str());
    }
    return converterMap[srcFormat].find(dstFormat)->second;
  }

}
