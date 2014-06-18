#ifndef Tribots_PROSILICA_H_
#define Tribots_PROSILICA_H_

#include <string>
#include "ImageSource.h"
#include "../../Structures/TribotsException.h"
#include "../../Fundamental/ConfigReader.h"
#include "YUVImage.h"

// actual implementation (mostly C)

extern "C"
{
#include "ProsilicaLib.h"
}

#define FRAMERATE 32

namespace Tribots {

  /**
   * PROSILICA camera driver.
   *
   * Static factory methods are used to provide controlled access to several
   * camera instances. getCamera() creates and manages all camera instances which
   * are connected to the system. This makes sure, that each IIDC object exists
   * only once.
   * On the first call of getCamera() for an UID, the camera with the specified UID
   * ist initialized and started. If no UID is specified, the first camera node is
   * used instead (backwards compatible with the singleton version of this class).
  */
  class Prosilica : public ImageSource {
    public:
  
      /** Constructor that reads relevant parameters from config-reader Arg1 using section Arg2; Arg3=force blocking mode */
      Prosilica (const ConfigReader&, const std::string&, bool =false) throw (HardwareException, InvalidConfigurationException);
      /** Destructor */
      ~Prosilica () throw ();

      /** Read the next image from the camera. 
       * This call blocks, until a new image has completely arrived at the camera
       * driver. In case of a problem with the camera this method returns a 
       * NULL pointer after about 200ms. To repair the camera, simply remove and
       * replug it and call this method again.
       * \return returns a pointer to an image buffer */
      ImageBuffer getImage() throw (ImageFailed);
      /** Bildbreite anfragen */
      int getWidth() const throw () { return CAM_WIDTH; }
      /** Bildhoehe anfragen */
      int getHeight() const throw () { return CAM_HEIGHT; }
      /** return the horizontal image offset, if any */
      int getOffsetX() const throw () { return 0; }
      /** return the vertical image offset, if any */
      int getOffsetY() const throw () { return 0; }

    protected:
      raw1394handle_t handle;
      dc1394_cameracapture camera;      

      unsigned char* Y, *U, *V;
      
  };

}


#endif
