#ifndef _JPEGIO_h_
#define _JPEGIO_h_

#include "ImageIO.h"

namespace Tribots {

  /** encapsulates libjpeg data structs to remove libjpeg headers from this
   *  public header. */
  class LibJPEGData; 

  class JPEGIO : public ImageIO {
  public:
    JPEGIO(int quality=50);
    virtual ~JPEGIO();

    virtual void write(const ImageBuffer& image, std::ostream& out) const
      throw(TribotsException);
    
    virtual ImageBuffer* read(ImageBuffer* image, std::istream& in) const
      throw(TribotsException);

    virtual std::string getDefEnding() const { return ".jpg"; }
    virtual std::string getTypeId() const { return "JPEG"; }
  protected:
    LibJPEGData *lj;
  };

}

#endif
