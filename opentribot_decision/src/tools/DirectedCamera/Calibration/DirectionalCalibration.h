
#ifndef _Tribots_DirectionalCalibration_h_
#define _Tribots_DirectionalCalibration_h_

#include <iostream>
#include <fstream>
#include <deque>
#include <opencv/cv.h>
#include <opencv/cvtypes.h>
#include "../../../ImageProcessing/Calibration/CameraOptics.h"
#include "../../../ImageProcessing/Formation/YUVImage.h"


namespace Tribots {

  /** Klasse, die den Zhang-Algorithmus zur Kamerakalibrierung
      aus opencv kapselt. */
  class DirectionalCalibration{
  public:
    /** Korrespondenzen von Welt-Bild-Punktpaeren */
    struct Correspondence {
      Vec world;
      Vec image;

      Correspondence () throw ();
      Correspondence (const Correspondence& c) throw ();
      Correspondence (Vec w, Vec i) throw ();
      const Correspondence& operator= (const Correspondence& c) throw ();
    };

    DirectionalCalibration () throw ();
    ~DirectionalCalibration () throw ();

    /** Die Kameraabbildungsklasse zurueckliefern, die berechnet wurde */
    const CameraOptics& getCameraOptics () const throw ();
    /** ein Bild entzerren; das erzeugte Bild muss explitit mit delete geloescht werden */
    Image* undistortImage (Image* src) throw (std::bad_alloc);
    /** ein Bild plattdurecken; das erzeugte Bild muss explitit mit delete geloescht werden */
    Image* flattenImage (Image* src) throw (std::bad_alloc);

    /** Bildbreite und -höhe setzen */
    void setImageDimensions (unsigned int w, unsigned int h) throw ();
    /** Punktkorrespondenzen fuer eine Ebene hinzufuegen */
    void addMarkers (const std::deque<Correspondence>&) throw (std::bad_alloc);
    /** Punktkorrespondenzen aus einer Datei lesen */
    void readMarkers (const char* filename) throw (std::bad_alloc);
    /** Kamerakalibrierung mit dem Zhang-Algorithmus. Die extrinsischen Parameter
        werden aus der letzten Ebene bestimmt. Zuvor muessen mindestens
        drei Ebenen mit addMarkers oder readMarkers uebergeben worden sein. */
    void calibrate () throw (std::invalid_argument, std::bad_alloc);

  private:
    std::deque<std::deque<Correspondence> > markers;
    CameraOptics cameraOptics;
    unsigned int width;
    unsigned int height;
  };

}
#endif
