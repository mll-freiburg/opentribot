#include "SharedMemSource.h"
#include "JPEGIO.h"
#include "PPMIO.h"
#include "ImageSourceFactory.h"
#include <fstream>
#include "../../Fundamental/stringconvert.h"

using namespace std;
using namespace Tribots;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::ImageSourceBuilder {
    public:
      Builder () {
        Tribots::ImageSourceFactory::get_image_source_factory ()->sign_up (string("SharedMemSource"), this);
      }
      Tribots::ImageSource* get_image_source (const std::string&, const Tribots::ConfigReader& reader, const std::string& section) throw (Tribots::TribotsException,std::bad_alloc) {
        return new Tribots::SharedMemSource (reader, section);
      }
  };
  Builder the_builder;
}




SharedMemSource::SharedMemSource (const std::string& filename) throw (HardwareException) {
  init (filename);
}

SharedMemSource::SharedMemSource(const ConfigReader& cfg, const std::string& section) throw (HardwareException, InvalidConfigurationException) :  imageIO(NULL), pos(0), width(0), height(0)
{
  string filename;
  if (cfg.get((section+"::filename").c_str(), filename) <= 0)
    throw InvalidConfigurationException((section+"::filename").c_str());
  init (filename);
}

void SharedMemSource::init (const std::string& filename) {
  ifstream in(filename.c_str());
  if (!in) {
    throw HardwareException(__FILE__ ": Could not open file.");
  }
  string type;
  in >> type;
  if (! (type == "single")) {
    throw HardwareException(__FILE__ ": This type is not supported.");
  }

  string ioType;
  in >> ioType;
  if (ioType=="JPEG")
    imageIO = new JPEGIO;
  else if (ioType=="PPM")
    imageIO = new PPMIO;
  else
    throw HardwareException((string(__FILE__)+string(": Unknown file type ")+ioType).c_str());

  int n;
  string imgFile;
  long timestamp;
  Time time;
  string filepath = Tribots::get_filepath (filename);
  while(in) {
    in >> n >> imgFile >> timestamp;
    time.set_msec(timestamp);
    images.push_back(FileInfo(n, filepath+Tribots::remove_filepath (imgFile), time));
  }
  in.close();

  if (images.size() == 0) {
    throw HardwareException(__FILE__ 
        ": File does not conatain any images.");
  }

  image = imageIO->read(0, images[0].filename); // momentan wird erstes bild
    // verschluckt (passiert auch in imageprocessing, oder???)
  width = image->width;
  height = image->height;

  // to be sure
  pos = 0;
}

SharedMemSource::~SharedMemSource() throw ()
{
  if (imageIO) {
    delete imageIO;
  }
  delete image;
}

ImageBuffer SharedMemSource::getImage() throw (HardwareException)
{
  image = getNextImage(image);
  return (*image);
}

ImageBuffer* SharedMemSource::getNextImage(ImageBuffer* image)
{
  pos = (pos+1) % images.size();
  image = imageIO->read(image, images[pos].filename);
  image->timestamp = (images[pos].timestamp);
  return image;
}

ImageBuffer* SharedMemSource::getPrevImage(ImageBuffer* image)
{
  pos = (pos-1+images.size()) % images.size();
  image = imageIO->read(image, images[pos].filename);
  image->timestamp = (images[pos].timestamp);
  return image;
}

string SharedMemSource::getFilename() const
{
  return images[pos].filename;
}


// Beispiel: Vektor enthält Bild 0 3 7 10
// Anfrage: Bild 3  -> liefert Bild 3
// Anfrage: Bild 5  -> liefert Bild 3
// Anfrage: Bild 16 -> liefert Bild 10
// Anfrage: Bild -1 -> liefert Bild 0  (!!!!!)
ImageBuffer* SharedMemSource::getImageNumber(int n, ImageBuffer* image)
{
  // dafür sorgen, dass pos aufs richtige oder auf ein späteres zeigt
  while (pos < (int)images.size()-1 && n > images[pos].n) {
    pos++;
  }
  // assertion:  n <= images[pos].n || pos == images.size()-1
  // dafür sorgen, dass pos aufs richtige (wenn da) oder auf das direkt 
  // davor liegende zeigt

  while (pos > 0 && n < images[pos].n) {
    pos--;
  }

  image = imageIO->read(image, images[pos].filename);
  image->timestamp = (images[pos].timestamp);
  return image;
}
