#ifndef _imagesource_h_
#define _imagesource_h_

#include "../../Structures/TribotsException.h"
#include "ImageBuffer.h"

namespace Tribots {

  /** Ausnahmenklasse, um das Ausbleiben eines Bildes zu beschreiben,
    z.B. bei Kameraabsturz oder bei nicht-blockierenden getImage()-Aufrufen */
  class ImageFailed : public HardwareException {
  public:
    ImageFailed () : HardwareException ("image unavailable") {;}
    ~ImageFailed () throw () {;}
  };

  /**
   * Interface defining a source of images. It is implemented by 
   * "drivers" for different devices like IEEE1394 cameras or 
   * video4linux frame grabbers. An ImageSource always returns
   * a plain image buffer. To get higher level access methods,
   * it is possible to wrap implementations of the Image Interface
   * around these plain buffers.
   */
  class ImageSource 
  {
  public:
    virtual ~ImageSource () {;}
    
    /**
     * get the next image from the image source. No image should 
     * be returned more than once.
     *
     * \returns an ImageBuffer holding the plain image. The memory
     *          of the ImageBuffer returned is owned by the image
     *          source an may change during the next call of getImage
     */
    virtual ImageBuffer 
    getImage() throw (HardwareException, TribotsException, ImageFailed)=0;

    /**
     * get the width of the images that this image source returns
     */
    virtual int getWidth() const=0;
    /**
     * get the height of the images that this image source returns
     */
    virtual int getHeight() const=0;
    /** return the horizontal image offset, if any */
    virtual int getOffsetX() const throw () { return 0; }
    /** return the vertical image offset, if any */
    virtual int getOffsetY() const throw () { return 0; }
    /** 
     * get the delay in ms of the image source
     */
    virtual int getDelay () const throw () { return 0; }

    /** define Balance Area for camera */
    virtual void setBalanceArea(int x, int y, int w, int h){return ;}
    virtual void getBalanceArea(int *x, int *y, int *w, int *h){return ;}


  };
};


#endif /* _imagesource_h_ */
