#ifndef _sharedmemwriter_h_
#define _sharedmemwriter_h_

#include "ImageMonitor.h"
#include "ImageIO.h"
#include <string>
#include <fstream>
#include <qsharedmemory.h>


namespace Tribots {

  /** Bilder-Wegspeicherer-Klasse; erlaubt z.Z. PPM- und JPEG-Formate */
  class SharedMemWriter : public ImageMonitor {
  public:
    /** Argumente:
      arg1 deie Spritiecherklasse,
      arg2: Dateiname der Logdatei
      arg3: jedes wievielte Bild wegspeichern?
      arg4: alles in eine Datei speichern (true) oder jedes Bild in eigene Datei (false) */
    SharedMemWriter(ImageIO* imageIO, const std::string& filename,
                int step=1, bool singleFile=true);
    virtual ~SharedMemWriter();
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
QSharedMemory *sharedMemory;


  };

}

#endif





