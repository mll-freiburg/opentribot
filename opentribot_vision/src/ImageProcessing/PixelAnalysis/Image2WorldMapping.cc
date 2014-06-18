
#include "Image2WorldMapping.h"
#include "../../Structures/TribotsException.h"
#include "../../Structures/Journal.h"
#include <sstream>
#include <fstream>

using namespace Tribots;
using namespace std;

Line3D ImageWorldLUTMapping::map3D(const Vec& vec) const
{
  int pos = static_cast<int>(vec.x)+static_cast<int>(vec.y)*width;
  if (pos < 0 || pos > lutSize) {
    stringstream inout;
    inout << "NAN_ERROR: given vec out of bounds in ImageWorldLUTMapping::map3D: " << vec << endl;
    string line;
    getline (inout, line);
    JERROR (line.c_str());
    return Line3D();
  }
  return lut3d[pos];
}

Vec ImageWorldLUTMapping::map(const Vec& vec) const
{
  int pos = static_cast<int>(vec.x)+static_cast<int>(vec.y)*width;
  if (pos < 0 || pos > lutSize) {
    stringstream inout;
    inout << "NAN_ERROR: given vec out of bounds in ImageWorldLUTMapping::map:  " << vec << endl;
    string line;
    getline (inout, line);
    JERROR (line.c_str());
    return Vec(0.,0.);
  }
  return lut[pos];
}

ImageWorldLUTMapping::ImageWorldLUTMapping(int width, int height, Vec3D origin)
: ImageWorldMapping(), origin(origin), lut(0), lut3d(0)
{
  resize (width, height);
}

ImageWorldLUTMapping::ImageWorldLUTMapping(const ImageWorldMapping& map) :
    lut(NULL), lut3d(NULL) {
  convert(map);
}

void ImageWorldLUTMapping::resize (int nwidth, int nheight) {
  width=nwidth;
  height=nheight;
  if (lut) delete [] lut;
  if (lut3d) delete [] lut3d;
  lut = new Vec[width*height];
  lut3d = new Line3D[width*height];
  lutSize=width*height;
}

ImageWorldLUTMapping::ImageWorldLUTMapping(std::string filename)
: ImageWorldMapping(), lut(0), lut3d(0), width(0), height(0), lutSize(0)  //Dirty: Initializes superclass with 1x1 lut
{
  load(filename);
}

ImageWorldLUTMapping::~ImageWorldLUTMapping()
{
  if (lut !=0) {
    delete [] lut;
  }
  if (lut3d != 0) {
    delete[] lut3d;
  }
}


void
ImageWorldLUTMapping::save(std::string filename) const
{
  ofstream out (filename.c_str(), ios::binary);
  if (!out) {
    throw TribotsException(__FILE__ ": Could not write mapping to disk.");
  }
  out << width << " " << height << '\n';
  // position of first '\n' will be checked during read operation

  out.write(reinterpret_cast<const char*>(lut),     // no problem with the current(!)
            sizeof(Vec) * width * height);    // implementation of Vec (\todo)

  out.write(reinterpret_cast<const char*>(lut3d),   // save out 3d LUT
            sizeof(Line3D) * width * height);

  out.write(reinterpret_cast<const char*>(&origin),
            sizeof(Vec3D));

  out.close();
}

void
ImageWorldLUTMapping::load(std::string filename)
{
  ifstream in (filename.c_str(), ios::binary);
  if (!in) {
    throw TribotsException(__FILE__ ": Could not read mapping from disk.");
  }
  if (lut !=0) {
    delete lut;
  }

  if (lut3d != 0) {
    delete lut3d;
  }

  char c;
  in >> width;
  in >> height;
  in.get(c);       // in >> c won't work here!
  if (c != '\n') {
    throw TribotsException(__FILE__ 
                           ": Distance lookuptable has wrong format.");
  }

  lut = new Vec[width*height];
  lut3d = new Line3D[width*height];
  lutSize = width*height;

  in.read(reinterpret_cast<char*>(lut),      // with current implementatin of Vec
          sizeof(Vec) * width * height);     // a binary read is possible

  in.read(reinterpret_cast<char*>(lut3d),      // with current implementatin of Vec
          sizeof(Line3D) * width * height);     // a binary read is possible

  in.read(reinterpret_cast<char*>(&origin),  // One additional 3D Vector for origin.
          sizeof(Vec3D));

  in.close();
}

void ImageWorldLUTMapping::set(int x, int y, const Vec& vec)
{
  lut[x+y*width] = vec;
}

void ImageWorldLUTMapping::set3D(int x,int y, const Line3D& line)
{
  lut3d[x+y*width] = line;
}

void ImageWorldLUTMapping::convert(const ImageWorldMapping& map)
{
  resize (map.getWidth(), map.getHeight());
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      set(x,y, map.map(Vec(x,y)));
      set3D(x,y, map.map3D(Vec(x,y)));
    }
  }
}
