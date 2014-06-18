
#ifndef _TribotsTools_CalibrationRecorder_h_
#define _TribotsTools_CalibrationRecorder_h_

#include <deque>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>
#include "../../../ImageProcessing/Calibration/Pixelset.h"
#include "../../../ImageProcessing/Formation/IIDC.h"
#include "../../components/ImageWidget.h"
#include "CalibrationRecorderControlPanel.h"

namespace TribotsTools {

  struct CalibrationMarker {
    Tribots::Vec truePosition;
    Tribots::Vec imagePosition;
    unsigned int imageNumber;
  };

  class CalibrationRecorder {
    enum Mode {
      neutral,
      redScreen,
      greenScreen,
      blueScreen,
      redMarker,
      blackMarker
    };
    Tribots::Pixelset regionOfInterest;
    Tribots::Pixelset* regionBefore;
    Tribots::Pixelset* regionWork;
    Tribots::Pixelset* regionAfter;
    Tribots::Vec gravityCenter;
    Tribots::Vec trueMarkerPosition;

    Tribots::IIDC* camera;

    std::deque<CalibrationMarker> markers;
    CalibrationMarker lastMarker;
    bool lastMarkerValid;
    unsigned int imageNumber;
    Mode mode;
    Mode oldMode;
    Image* image;
    Image* oldImage;
    unsigned int px;
    unsigned int py;
    unsigned int imageWidth;
    unsigned int imageHeight;

    int socketDescriptor;
    unsigned short int serverPort;
    struct sockaddr_in serverAddress;
    struct hostent *hostInfo;

    TribotsTools::ImageWidget* widget;
    CalibrationRecorderControlPanel* panel;
    std::ostream* markerout;

    void communicate ();  ///< mit MarkerDisplay-Programm kommunizieren und warten, bis Antwort gekommen
    void captureImage ();  ///< mehrere Bilder grabben und letztes in 'image' speichern
    void processImage ();  ///< Bild verarbeiten
    void displayImage ();  ///< Bild anzeigen
    bool nextStep ();  ///< Umschalten zum naechsten Verarbeitungsschritt

  public:
    CalibrationRecorder (const Tribots::ConfigReader& cfg, const char* hostname, unsigned int port);
    ~CalibrationRecorder () throw ();

    void loop ();
    void writeMarkers (std::ostream&);
  };

}

#endif
