#ifndef _FILESOURCE_H_
#define _FILESOURCE_H_

#include "ImageSource.h"
#include "ImageIO.h"
#include <string>
#include <vector>
#include "../../Fundamental/Time.h"
#include "../../Fundamental/ConfigReader.h"

namespace Tribots {

  class FileSource : public ImageSource
  {
  public:
    /** reads relevant parameters from config-reader Arg1 using section Arg2 */
    FileSource(const ConfigReader&, const std::string&) throw (HardwareException, InvalidConfigurationException);
    /** reads images from File Arg1 */
    FileSource(const std::string&) throw (HardwareException);
    virtual ~FileSource() throw ();
    
    /** implements the ImageSource. Uses getNextImage to give the
     *  next image of the stream. At the end of the stream restarts
     *  with the first image. */
    virtual ImageBuffer getImage() throw (HardwareException);
    
    virtual ImageBuffer* getNextImage(ImageBuffer* image=0); 
    virtual ImageBuffer* getPrevImage(ImageBuffer* image=0);

    virtual ImageBuffer* getImageNumber(int n, ImageBuffer* image=0); 
    /** returns the filename of the present image */
    virtual std::string getFilename() const;
    
    virtual int getWidth() const { return width; }
    virtual int getHeight() const { return height; }
    
  protected:
    void init (const std::string&);
    struct FileInfo {
      int n;
      std::string filename;
      Time timestamp;
      FileInfo(int n, const std::string& filename, const Time& timestamp)
	: n(n), filename(filename), timestamp(timestamp) {}
    };

    ImageIO* imageIO;
    int pos;
    std::vector<FileInfo> images;
    
    int width;
    int height;
    
    ImageBuffer* image;  ///< merkt sich das Bild, muss sich um den Speicher kümmern
  };
}

#endif
