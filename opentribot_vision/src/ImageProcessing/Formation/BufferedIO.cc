
#include "BufferedIO.h"
#include <fstream>
#include <sstream>

using namespace std;
using namespace Tribots;


  
BufferedIO::BufferedIO(int n, ImageIO* io) : n(n), io(io) 
{}

BufferedIO::~BufferedIO()
{
  writeBuffers();
  if (io) delete io;
}
  
void BufferedIO::writeBuffers()
{
    cerr << "writing buffered images..." << endl; 
	for (unsigned int i=0; i < filenames.size(); i++) {
    cerr << filenames[i] << endl;
    ofstream out(filenames[i].c_str());
    if (!out) {
      throw TribotsException("trying to write buffered data but could not open file for writing");
    }
    out << bufferedData[i] << endl;
    out.close();
  }
  filenames.clear();
  bufferedData.clear();
}

void BufferedIO::write(const ImageBuffer& image, const std::string& filename) const throw(TribotsException)
{
  BufferedIO* tp = const_cast<BufferedIO*>(this);
  tp->filenames.push_back(filename);
  stringstream str;
  io->write(image, str);
  tp->bufferedData.push_back(str.str());
  if (filenames.size() >= (unsigned int)n) {
    tp->writeBuffers();
  }
}
  
void BufferedIO::write(const ImageBuffer& image, std::ostream& out) const throw(TribotsException)
{
  io->write(image, out);
}
ImageBuffer* BufferedIO::read(ImageBuffer* image, const std::string& filename) const throw(TribotsException)
{
  return io->read(image, filename);
}

ImageBuffer* BufferedIO::read(ImageBuffer* image, std::istream& in) const throw(TribotsException)
{
  return io->read(image, in);
}
  
string BufferedIO::getDefEnding() const {
  return io->getDefEnding();
}

string BufferedIO::getTypeId() const 
{
  return io->getTypeId();
}
