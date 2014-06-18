#ifndef _Tribots_BufferedIO_h_
#define _Tribots_BufferedIO_h_

#include "ImageIO.h"
#include "PPMIO.h"
#include <vector>

namespace Tribots {

  
  /**
   * Implementierung des ImageIO - Interfaces, die Bilder zwischenpuffert und
   * auf einen Rutsch schreibt, sofern man diese Klasse mit einem Dateinamen
   * aufruft.
   */
  class BufferedIO : public ImageIO {
  public:
    BufferedIO(int n=100, ImageIO* io = new PPMIO); 
    virtual ~BufferedIO();
      
    virtual void write(const ImageBuffer& image, const std::string& filename) const throw(TribotsException);
    virtual void write(const ImageBuffer& image, std::ostream& out) const throw(TribotsException);
    virtual ImageBuffer* read(ImageBuffer* image, const std::string& filename) const throw(TribotsException);
    virtual ImageBuffer* read(ImageBuffer* image, std::istream& in) const throw(TribotsException);

    virtual std::string getDefEnding() const;
    virtual std::string getTypeId() const;
    
  protected:
    std::vector<std::string> filenames;
    std::vector<std::string> bufferedData;
    int n;
    ImageIO* io;
    
    void writeBuffers();
  };
};

#endif
