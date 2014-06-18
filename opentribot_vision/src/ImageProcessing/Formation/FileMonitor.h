#ifndef _filemonitor_h_
#define _filemonitor_h_

#include "ImageMonitor.h"
#include "ImageIO.h"
#include <string>
#include <fstream>

namespace Tribots {

  /** Bilder-Wegspeicherer-Klasse; erlaubt z.Z. PPM- und JPEG-Formate */
  class FileMonitor : public ImageMonitor {
  public:
    /** Argumente:
      arg1: die Spiecherklasse,
      arg2: Dateiname der Logdatei
      arg3: jedes wievielte Bild wegspeichern?
      arg4: alles in eine Datei speichern (true) oder jedes Bild in eigene Datei (false) */
    FileMonitor(ImageIO* imageIO, const std::string& filename, 
		int step=1, bool singleFile=true);
    virtual ~FileMonitor();
    virtual void monitor(const ImageBuffer&, 
			 const Time&, const Time&);

  protected:
    ImageIO* imageIO;
    std::ofstream* logOut;
    std::ofstream* imgOut;
    std::string filename;
    bool singleFile;

    int counter;
    int step;
  };

}

#endif
