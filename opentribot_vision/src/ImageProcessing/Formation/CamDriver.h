#ifndef Tribots_CamDriver_H_
#define Tribots_CamDriver_H_

#include <string>
#include <libraw1394/raw1394.h>
#include <dc1394/control.h>
#include <vector>
#include "IIDC.h"


namespace Tribots {

/**
 * camera driver.
 *
 * Static factory methods are used to provide controlled access to several
 * camera instances. getCamera() creates and manages all camera instances which
 * are connected to the system. This makes sure, that each IIDC object exists
 * only once.
 * On the first call of getCamera() for an UID, the camera with the specified UID
 * ist initialized and started. If no UID is specified, the first camera node is
 * used instead (backwards compatible with the singleton version of this class).
 */
class CamDriver {

public:
  /** nach Kameras auf dem Bus suchen und eine Liste mit Kamera-UIDs liefern */
  static std::vector<std::string> getCameraUIDs () throw (HardwareException);
  /**
   * CamDriver Camera factory. Use this method to get hold of the IIDC_DMA instance.
   * On the first call, all cameras at any port are initialized and stored into
   * the multicam vector. The Camera instance specified by the given arguments
   * (esp. its UID number) is started and retruned. On subsequent calls given
   * framerate and format arguments are ignored and a pointer to the already
   * running instance is returned.
   *
   * \attention Never call delete on the returned pointer! This will cause
   *            a segmentation fault, if you try to get a new instance later.
   *            Use IIDC_DMA::destroy_camera() instead.
   *
   * @param device_name video1394 device
   * @param port The port to use. !! Ignored (all ports are scanned), included
   *             for backward compatibility only !!
   * @param mode string that describes the desired camera mode
   * @param uid the uid number of the requested device as hex-string.
   *            This is the same as the UID string in the CAMERA INFO message.
   *            Default: first camera on bus
   * @param delay the average delay of the camera in ms
   */
  static CamDriver* getCamera(const char* device_name= "/dev/video1394/0",
                         int =0,  // port
                         const std::string& mode ="",
                         const std::string& uid = "",
                         bool blocking = 1,
                         int delay = 0) throw (HardwareException);

  /**
   * Destroys all existing instances of the IIDC_DMA class. If theres no
   * instance, it does nothing.
   * Use this method instead of the camera's destructor.
   */
  static void destroyAllCamera() throw ();
  /** Destroys a single camera given in Arg1; The pointer Arg1 beocomes invalid thereby */
  static void destroyCamera (CamDriver*) throw ();

  void initSoftBalance(bool doSoftExposure, bool doSoftWhiteBalance, 
                       bool adjustGain = false, int balanceStep = 1,
                       unsigned char exposure = 127,
                       int shutterLogic = +1,
                       unsigned int shutterMin = 0, 
                       unsigned int shutterMax = 0,
                       unsigned int gainMin = 0, 
                       unsigned int gainMax = 0,
                       unsigned int uvMin = 0, 
                       unsigned int uvMax = 0);  

  /** Read the next image from the camera. 
   * This call blocks, until a new image has completely arrived at the camera
   * driver. In case of a problem with the camera this method returns a 
   * NULL pointer after about 200ms. To repair the camera, simply remove and
   * replug it and call this method again.
   * \return returns a pointer to an image buffer */
  ImageBuffer getImage() throw (ImageFailed);

  /** Bildbreite anfragen */
  int getWidth() const throw ();
  /** Bildhoehe anfragen */
  int getHeight() const throw ();
  /** Bild-Offset in x-Richtung abfragen */
  int getOffsetX() const throw ();
  /** Bild-Offset in y-Richtung abfragen */
  int getOffsetY() const throw ();

  /** Kamera-Typ anfragen */
  std::string getCameraType () throw ();
  /** Kamera-Modus anfragen */
  std::string getCameraMode () throw ();
  /** Kamera-UID anfragen */
  std::string getCameraUID () const throw ();
  /** die angenommene Kameraverzoegerung in ms */
  int getDelay () const throw ();
  
  /** loads the camera settings from the specified memory channel; returns: success? */
  bool loadSettings(int channel = 1) throw ();
  /** saves the camera settings to the specified memory channel; returns: success? */
  bool saveSettings(int channel = 1) throw ();
  /** return number of available memory channels */
  unsigned int numChannel() throw ();
  
  /** den Wert eines Kamerafeatures abfragen.
    \attention fuer den Weissabgleich die Methode getWhiteBalance(.) nutzen */
  unsigned int getFeatureValue (IIDC::CameraFeature) throw ();
  /** den Wert eines Kamerafeatures setzen, sofern das Feature verfügbar und manuell verstellbar ist.
    \attention fuer den Weissabgleich die Methode setWhiteBalance(.) nutzen */
  bool setFeatureValue (IIDC::CameraFeature, unsigned int) throw ();
  /** den Modus eines Kamerafeatures abfragen. Hier auch whiteBalance moeglich */
  IIDC::CameraFeatureMode getFeatureMode (IIDC::CameraFeature) throw ();
  /** den Modus eines Kamerafeatures setzen. Hier auch whiteBalance moeglich */
  bool setFeatureMode (IIDC::CameraFeature, IIDC::CameraFeatureMode) throw ();
  /** die moeglichen Modi eines Kamerafeatures anfragen. Hier auch whiteBalance moeglich.
    (Returnwert & feaure) != 0, wenn Modus verfuegbar */
  unsigned int availableFeatureModes (IIDC::CameraFeature) throw ();
  /** minimalen und maximalen Wert eines Camerafeatures anfragen. Hier auch whiteBalance moeglich.
    Arg1 (return): Minimalwert,
    Arg2 (return): Maximalwert,
    Arg3: Feature */
  bool getFeatureMinMaxValue (unsigned int&, unsigned int&, IIDC::CameraFeature) throw ();
  /** Weissabgleich setzten u-Wert, v-Wert */
  bool setWhiteBalance (unsigned int, unsigned int) throw ();
  /** Werte des Weissabgleichs anfragen u-Wert, v-Wert */
  bool getWhiteBalance (unsigned int&, unsigned int&) throw ();

  /** toggle software auto-exposure on or off */
  void toggleSoftwareExposure(bool on) throw();
  bool isSoftwareExposure() const throw();
  /** toggle software white-balance on or off */
  void toggleSoftwareWhiteBalance(bool on) throw();
  bool isSoftwareWhiteBalance() const throw();

  virtual void setBalanceArea(int x, int y, int w, int h);
  virtual void getBalanceArea(int *x, int *y, int *w, int *h) const;

  /** check whether camera works, write error messages into arg1, return true if camera is okay */
  bool checkCamera(std::ostream&) throw ();
    
 protected:
   /** Struktur, um Kameraknoten auf dem Bus zu repraesentieren */
  struct DCInfo {
    nodeid_t node;
    std::string uid;
    int port;
    int channel;
  };
  
  /** Struktur, um Kamerafeatures zu repraesentieren fuer den Fall, dass sie nach einem Reset wieder gesetzt werden muessen */
  struct DCFeature {
    IIDC::CameraFeature feature;
    IIDC::CameraFeatureMode mode;
    unsigned int value;
    unsigned int second_value;  // fuer Weissabgleich
  };
  
  bool doSoftExposure;       ///< whether or not to control exposure by software
  bool doSoftWhiteBalance;   ///< whether or not to control white balance by software
  bool adjustGain;           ///< whether or not to use gain parameter when controling exposure
  
  int balanceStep;
  unsigned int software_exposure;
  unsigned int gainMin, gainMax;
  unsigned int shutterMin, shutterMax;
  unsigned int uvMin, uvMax;
  int shutterLogic;
  double autoExposureDelta;
  double autoExposureError;

  /** Control exposure (shutter, gain and brightness) and white balance by software. */
  void doSoftwareBalance();  
   
  /// Static variables that are used by all instances :
  static std::vector<DCInfo> cameranodes;  ///< holds a list of available camera nodes on the firewire bus
  static std::vector<CamDriver*> multicam;    ///< holds started camera instances
  static unsigned int bandwidth_used;    ///< holds the bandwidth used by all running cameras so far
  static unsigned int reset_count;       ///< counts multiple calls to the reset function.
  static int num_ports;                  ///< number of ports on this machine
  static int numNodes;                   ///< number of raw1394-nodes

  
  /// Instance fields of a single camera:
  dc1394_cameracapture camera;           ///< dc1394-camera struct
  raw1394handle_t handle;                ///< camera handle
  int firstFrame;                        ///< true before first frame has been captured
  int failure;                           ///< increases with consequent failures

  bool initDone;                         ///< wurde dc1394_init_camera ausgefuehrt?
  bool dmaInitDone;                      ///< wurde dc1394_dma_setup_capture ausgefuehrt?
  bool startDone;                        ///< wurde dc1394_start_iso_transmission ausgefuehrt?
  
  int frameCount;                        ///< frame counter
  
  std::string device_name;               ///< remembers the given device name for a reset
  int port;                              ///< port number
  IIDC::CameraFormat cameraFormat;             ///< camera format
  IIDC::ColorCoding colorCoding;               ///< color coding
  float framerate;                       ///< frame rate in frames per second
  unsigned int imageWidth;               ///< image width
  unsigned int imageHeight;              ///< image height
  unsigned int offsetX;                  ///< horizontal offset of image
  unsigned int offsetY;                  ///< vertical offset of image
  int mode_dc1394;                       ///< format constant in dc1394 style
  int framerate_dc1394;                  ///< framerate constant in dc1394 style (not for format7)
  int channel;                           ///< dma channel
  std::string uid;                       ///< uid
  bool isBlocking;                       ///< specifies if the camera uses blocking or  polling capture mode.
  const int camera_delay;                ///< expected camera delay in ms
  unsigned int bandwidth_cam;            ///< bandwidth used by this camera
  std::vector<DCFeature> feature_set;    ///< die Werte der gesetzten Kamera-Features

  int bAX;                               ///< Balance area Position
  int bAY;                               ///< Balance area Position
  int bAWidth;                           ///< Balance area Breite
  int bAHeight;                          ///< Balance area Hoehe

  
  /** (re-) init 'cameranodes' looking for all cameras on the bus */
  static void searchCameraNodes () throw (HardwareException);
  
  /** returns the cameranode with the demanded uid, if available, NULL otherwise. If uid="" the first camera is returned */
  static const DCInfo* getCameraNode (const std::string& uid) throw ();
  
  /** completely restart a camera, i.e. look for the correct camera node, initialize the camera, 
  initialize dma_capture and start ISO transmission */
  IIDC::CameraError restartCamera () throw ();
  
  /** Create a certain camera object. Does not start transmission.
    \attention never call the constructor by hand, use getCamera(.) instead. */
  CamDriver (const char* device_name, const std::string& uid, bool blocking, int delay) throw ();

  /** Initialize camera object; used by restartCamera(). Does not start transmission */
  IIDC::CameraError init() throw ();
  
  /** release camera */
  IIDC::CameraError deinit () throw ();

  /** starts dma_capture and ISO transmission */
  IIDC::CameraError startTransmission() throw ();

  /** set feature values saved in 'features' */
  void setSavedCameraFeatures() throw ();
  
  /** stop ISO transmission and unlisten dma; inverse to startTransmission */
  IIDC::CameraError stopTransmission() throw ();
  
  /** roughly estimate the necessary bandwidth in bit/sec for the given color coding, framerate and image size */
  unsigned int estimateBandwidth(IIDC::ColorCoding cc, float framerate, unsigned int iw, unsigned int ih) throw ();

  /** return necessary byte per pixel for a certain color coding */
  static float bytePerPixel (IIDC::ColorCoding cc) throw ();

  /** calculate the 'most similar' available camera format for given demands.
    Arg1: return the mode constant for libdc1394
    Arg2: return the framerate constant for libdc1394
    Arg3: the demanded camera format
    Arg4: the demanded color format
    Arg5: the demanded framerate, returns the most similar possible framerate
    Arg6: the demanded image width, returns the most similar possible image width
    Arg7: the demanded image height, returns the most similar possible image height */
  void availableFormat (int& mode_dc1394, int& framerate_dc1394, IIDC::CameraFormat cf, IIDC::ColorCoding cc, float& fps, unsigned int& iw, unsigned int& ih, unsigned int& offsetX, unsigned int& offsetY) throw ();
  
  /** extract camera format (arg1), color coding (arg2), framerate (arg3), image width (arg4), image height (arg5) from a mode description string (arg6);
  return true, if (arg6) is a valid mode description string */
  static bool revealCameraMode (IIDC::CameraFormat&, IIDC::ColorCoding&, float&, unsigned int&, unsigned int&, unsigned int&, unsigned int&, const std::string&) throw ();
  
  /** query the camera uid of the camera given by 'handle' and 'node'; returns an hex-string */
  static std::string getCameraUID (raw1394handle_t handle, nodeid_t node) throw ();

  
  
public: /// public methods but none the less internals of the IIDC class
  
  /** Destructor. Is called by destroyCamera via delete. */
  virtual ~CamDriver() throw ();
  
  /** Static bus reset handler that is called by the
   * __iidc_bus_reset_handler_static() every time there was a bus reset (camera
   * plugged or unplugged). Finds the existing camera with the right handle
   * and calls its instance resetHandler().
   *
   * There should be no reason to ever call this method manually.
   */
  static int bus_reset_handler_static(raw1394handle_t handle,
                                      unsigned int generation) throw ();


  /**
   * Extern C callback function that is used as bus reset handler and passed
   * to the raw1394 driver. This function simply calls the reset handler
   * of the IIDC singleton (if there is no instance, it creates one).
   */
  friend int  __iidc_bus_reset_handler_static(raw1394handle_t, unsigned int);
      
};

}


#endif
