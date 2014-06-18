#ifndef Tribots_IIDC_H_
#define Tribots_IIDC_H_

#include <string>
#include "ImageSource.h"
#include "../../Structures/TribotsException.h"
#include "../../Fundamental/ConfigReader.h"

namespace Tribots {

  class CamDriver;

  /**
   * IIDC camera driver.
   *
   * Static factory methods are used to provide controlled access to several
   * camera instances. getCamera() creates and manages all camera instances which
   * are connected to the system. This makes sure, that each IIDC object exists
   * only once.
   * On the first call of getCamera() for an UID, the camera with the specified UID
   * ist initialized and started. If no UID is specified, the first camera node is
   * used instead (backwards compatible with the singleton version of this class).
  */
  class IIDC : public ImageSource {
    public:
      /** Liste mit Farbcodierungen der Kameras */
      enum ColorCoding {
        YUV411,
        YUV422,
        YUV444,
        RGB8,
        RGB16,
        Mono8,
        Mono16
      };

      /** Liste der moeglichen Uebertragungsformate (Kameramodi) */
      enum CameraFormat {
        Format0,
        Format7
      };

      /** Liste einstellbarer Kamerafeatures */
      enum CameraFeature {
        brightness,
        exposure,
        sharpness,
        whiteBalance,
        hue,
        saturation,
        gamma,
        shutter,
        gain,
        filter
      };
  
      /** Liste mit Featuremodi */
      enum CameraFeatureMode {
        featureUnavailable = 1,
        featureOff = 2,
        featureMan = 4,
        featureAuto = 8
      };

      /** Fehlerursachen */
      enum CameraError {
        successful,            ///< erfolgreiche Operation
        unknownReason,       ///< unspezifizierter Fehler
        cameraNotFound,     ///< Kameraknoten nicht gefunden
        crowdedBus,        ///< Firewirebus zu voll
        initFailed,             ///< init fehlgeschlagen
        dmaInitFailed,      ///< dma-Init fehlgeschlagen
        startTransmissionFailed   ///< ISO-Uebertragung Start fehlgeschlagen
      };
  
      /** Constructor that reads relevant parameters from config-reader Arg1 using section Arg2; Arg3=force blocking mode */
      IIDC (const ConfigReader&, const std::string&, bool =false) throw (HardwareException, InvalidConfigurationException);
      /** Destructor */
      ~IIDC () throw ();

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
      /** return the horizontal image offset, if any */
      int getOffsetX() const throw ();
      /** return the vertical image offset, if any */
      int getOffsetY() const throw ();

      /** Kamera-Typ anfragen */
      virtual std::string getCameraType () throw ();
      /** Kamera-Modus anfragen */
      virtual std::string getCameraMode () throw ();
      /** Kamera-UID anfragen */
      virtual std::string getCameraUID () const throw ();
      /** die angenommene Kameraverzoegerung in ms */
      virtual int getDelay () const throw ();

      /** loads the camera settings from the specified memory channel; returns: success? */
      virtual bool loadSettings(int channel = 1) throw ();
      /** saves the camera settings to the specified memory channel; returns: success? */
      virtual bool saveSettings(int channel = 1) throw ();
      /** return number of available memory channels */
      virtual unsigned int numChannel() throw ();

      /** den Wert eines Kamerafeatures abfragen.
      \attention fuer den Weissabgleich die Methode getWhiteBalance(.) nutzen */
      virtual unsigned int getFeatureValue (CameraFeature) throw ();
      /** den Wert eines Kamerafeatures setzen, sofern das Feature verfügbar und manuell verstellbar ist.
      \attention fuer den Weissabgleich die Methode setWhiteBalance(.) nutzen */
      virtual bool setFeatureValue (CameraFeature, unsigned int) throw ();
      /** den Modus eines Kamerafeatures abfragen. Hier auch whiteBalance moeglich */
      virtual CameraFeatureMode getFeatureMode (CameraFeature) throw ();
      /** den Modus eines Kamerafeatures setzen. Hier auch whiteBalance moeglich */
      virtual bool setFeatureMode (CameraFeature, CameraFeatureMode) throw ();
      /** die moeglichen Modi eines Kamerafeatures anfragen. Hier auch whiteBalance moeglich.
      (Returnwert & feaure) != 0, wenn Modus verfuegbar */
      virtual unsigned int availableFeatureModes (CameraFeature) throw ();
      /** minimalen und maximalen Wert eines Camerafeatures anfragen. Hier auch whiteBalance moeglich.
        Arg1 (return): Minimalwert,
        Arg2 (return): Maximalwert,
        Arg3: Feature */
      virtual bool getFeatureMinMaxValue (unsigned int&, unsigned int&, CameraFeature) throw ();
      /** Weissabgleich setzten u-Wert, v-Wert */
      virtual bool setWhiteBalance (unsigned int, unsigned int) throw ();
      /** Werte des Weissabgleichs anfragen u-Wert, v-Wert */
      virtual bool getWhiteBalance (unsigned int&, unsigned int&) throw ();

      /** Software-Weissabgleich und Software-Shutter */
      virtual void initSoftBalance(bool doSoftExposure, bool doSoftWhiteBalance, 
                                   bool adjustGain = false, int balanceStep = 1,
                                   unsigned char exposure = 127,
                                   int shutterLogic = 1,
                                   unsigned int shutterMin = 0,
                                   unsigned int shutterMax = 0,
                                   unsigned int gainMin = 0,
                                   unsigned int gainMax = 0,
                                   unsigned int uvMin = 0,
                                   unsigned int uvMax = 0);
      /** toggle software auto-exposure on or off */
      virtual void toggleSoftwareExposure(bool on) throw();
      virtual bool isSoftwareExposure() const throw();
      /** toggle software white-balance on or off */
      virtual void toggleSoftwareWhiteBalance(bool on) throw();
      virtual bool isSoftwareWhiteBalance() const throw();
      virtual void setBalanceArea(int x, int y, int w, int h);
      virtual void getBalanceArea(int *x, int *y, int *w, int *h) const;

      /** pruefen, ob Kamera richtig funktioniert: Schwarz/Weiss-Modus, richtig gesetzte Kamera-Feature,
          schreibe Fehlerbericht in arg1, liefere true, wenn alles okay ist */
      virtual bool checkImageSource (std::ostream&) throw ();
    protected:
      CamDriver* cam_driver;
  };

}


#endif
