#include "FileMonitor.h"

#include <sstream>

namespace Tribots {

  using namespace std;
  
  FileMonitor::FileMonitor(ImageIO* imageIO, const std::string& filename,
			   int step, bool singleFile)
    : ImageMonitor(), imageIO(imageIO), logOut(0), imgOut(0), 
      filename(filename), singleFile(singleFile), counter(0), step(step)
  {
    logOut = new ofstream((filename+".log").c_str());

    if (singleFile) {
      string ending = imageIO->getDefEnding();
      string imgFilename = filename+ending+"_stream";
      imgOut = new ofstream(imgFilename.c_str());

      (*logOut) << "stream " << imageIO->getTypeId() 
		<< " " << imgFilename << endl;
    }
    else {
      (*logOut) << "single " << imageIO->getTypeId() << " " << endl;
    }    
  }

  FileMonitor::~FileMonitor()
  {
    if (logOut) {
      logOut->close();
      delete logOut;
    }
    if (imgOut) {
      imgOut->close();
      delete imgOut;
    }
    if (imageIO) {
      delete imageIO;
    }
  }

  void FileMonitor::monitor(const ImageBuffer& image, 
			    const Time&, const Time&)
  {
    if (counter % step != 0) {
      counter++;   // dieser Regelschleifenzaeler soll noch in die Architektur
      return;     
    }

    if (! *logOut) {
      throw TribotsException("Could not open image monitor log file.");
    }
    if (singleFile) {
      if (! *imgOut) {
	throw TribotsException("Could not open image monitor output file.");
      }
      (*imgOut) << image.timestamp << endl;
      imageIO->write(image, *imgOut);
      (*logOut) << counter++ << " " << image.timestamp << endl;
    }
    else {
      stringstream base;
      base << filename << image.timestamp << imageIO->getDefEnding();
      imageIO->write(image, base.str());
      (*logOut) << counter++ << " " << base.str() << " " 
		<< image.timestamp << endl;
    }
  }
}
