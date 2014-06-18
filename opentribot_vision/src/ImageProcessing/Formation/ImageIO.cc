#include "ImageIO.h"
#include <fstream>

using namespace Tribots;
using namespace std;

void ImageIO::write(const ImageBuffer& image, const std::string& filename) const throw(TribotsException)
{
  ofstream out(filename.c_str());
  if (!out) {
    throw TribotsException(__FILE__": could not open file for output.");
  }

  write(image, out);

  out.close();
}

ImageBuffer* ImageIO::read(ImageBuffer* image, const std::string& filename) const throw(TribotsException)
{
  ifstream in(filename.c_str());
  if (!in) {
    throw TribotsException(__FILE__": could not open file for input.");
  }
  ImageBuffer* img = read(image, in);
  in.close();
  return img;
}

