
#include "PPMIO.h"
#include "RGBImage.h"
#include <fstream>

using namespace std;
using namespace Tribots;

namespace {
  
    /* skips spaces and recursivly multi-line comments on a given stream.
    */
  static void skip(istream& in)
  {
    char c;
    do {
      if (!in.get(c)) return;
    } while(isspace(c) || c=='\n' || c=='\r');
    if (c=='#') {                  // start of a comment
      do {                         // look for the end of the line
        if (!in.get(c)) return;
      } while(c!='\n');
      skip(in);                    // skip again at start of next line.
    }
    else                           // c was neither comment(#) nor space
      in.putback(c);               // so better put it back,
  }

}


  
string PPMIO::getDefEnding() const 
{
  return ".ppm";
}

string PPMIO::getTypeId() const 
{
  return "PPM";
}

void PPMIO::write(const ImageBuffer& image, ostream& out) const
      throw(TribotsException)
{
  ImageBuffer& imgBuf = const_cast<ImageBuffer&>(image);

    // write header

  out << "P6" << endl;
  out << "# CREATOR: PPMIO.h, robotcontrol\n";
  out << image.width << " " << image.height << " " 
      << "255" << "\n";

    // write binary data
  if (image.format == ImageBuffer::FORMAT_RGB) {
    out.write(reinterpret_cast<char*>(imgBuf.buffer), imgBuf.size);
  }
  else {
    ImageBuffer newBuf(image.width, image.height, 
                       ImageBuffer::FORMAT_RGB, 
                       new unsigned char[image.width * 
                           image.height * 3],
                       image.width * image.height * 3);
    ImageBuffer::convert(imgBuf, newBuf);
    out.write(reinterpret_cast<char*>(newBuf.buffer), newBuf.size);
    delete [] newBuf.buffer;
  }    
}

ImageBuffer* PPMIO::read(ImageBuffer* image, istream& in) const throw(TribotsException)
{
    // read header
  int type, maxCol;
  int width, height;
  char c;

  skip(in);
  if (!in.get(c)) {
    throw TribotsException(__FILE__ ": Unexpected end of file.");
  }
  if (c != 'P' && c != 'p') {
    throw TribotsException(__FILE__ ": Not a PPM file.");    
  }
    
  in >> type;
  if (type != 6) {      // binary format?
    throw TribotsException(__FILE__ ": Not a binary PPM.");
  }

  skip(in);
  in >> width;
  skip(in);
  in >> height;
  skip(in);
  in >> maxCol;            // will be ignored

  in.ignore(1);

  if (image != 0) {
    if (image->width != width || image->height != height) {
      throw TribotsException(__FILE__ ": Given image of wrong dimensions. "
          "Use a NULL-pointer instead.");
    }
  }
  else {
    image = new ImageBuffer (width, height, ImageBuffer::FORMAT_RGB, new unsigned char [3*width*height], 3*width*height);
  }

    // read binary data
  if (image->format == ImageBuffer::FORMAT_RGB) {
    in.read(reinterpret_cast<char*>(image->buffer), 
            image->size);
  }
  else {
    ImageBuffer newBuf(width, height, 
                       ImageBuffer::FORMAT_RGB, 
                       new unsigned char[width * height * 3],
                       width * height * 3);
    in.read(reinterpret_cast<char*>(newBuf.buffer), newBuf.size);
    ImageBuffer::convert(newBuf, (*image));
      
    delete [] newBuf.buffer;
  }    
    
  return image;
}
