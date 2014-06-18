
#include "RobotMask.h"
#include <fstream>
#include <cstring>
using namespace std;
using namespace Tribots;


namespace {
  /* skips spaces and recursivly multi-line comments on a given stream. */
  void skip(istream& in)
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
    else {                          // c was neither comment(#) nor space
      in.putback(c);               // so better put it back,
    }
  }
}



RobotMask::RobotMask(const char* filename) throw (TribotsException)
  : mask(0), width(0), height(0)
{
  ifstream in(filename);
  if (!in)
    throw TribotsException ((string("Could not open mask file: ")+filename).c_str());

  char c;
  int type, maxCol;

  skip(in);
  if (!in.get(c)) 
    throw TribotsException (__FILE__
        " Unexpected end of file in RobotMask.");
  if (c != 'P' && c != 'p') 
    throw TribotsException (__FILE__
        " Not a PPM file in RobotMask.");

  skip(in);
  in >> type;
  skip(in);
  in >> width;
  skip(in);
  in >> height;
  skip(in);
  in >> maxCol;    // is ignored \todo

  if (type != 6) { // is binary format?
    throw TribotsException (__FILE__
        " Not a binary ppm file in RobotMask.");
  }
  in.ignore(1);    // \todo this assumes exactly one character before data...
  unsigned char* buf = new unsigned char[width*height*3];
  in.read(reinterpret_cast<char*>(buf), width*height*3);
  mask = new bool[width*height];

  for (unsigned int x = 0; x < width; x++) {
    for (unsigned int y = 0; y < height; y++) {
      mask[x+y*width] = buf[x*3+y*width*3] > 127;  // check red channel marked
    }
  }
  delete [] buf;
}


RobotMask::RobotMask (unsigned int w, unsigned int h, bool def) : width (w), height (h) 
{
  mask = new bool[width*height];
  bool* ptr = mask;
  for (unsigned int x = 0; x < width; x++)
    for (unsigned int y = 0; y < height; y++)
      (*ptr++)=def;
}


RobotMask::~RobotMask()
{
  if (mask) delete [] mask;
}


unsigned int RobotMask::getWidth () const throw () { return width; }

unsigned int RobotMask::getHeight () const throw () { return height; }

RobotMask::RobotMask (const RobotMask& src) throw () : mask (NULL)
{
  (*this) = src;
}

const RobotMask& RobotMask::operator= (const RobotMask& src) throw ()
{
  if (mask) delete [] mask;
  width= src.width;
  height = src.height;
  mask = new bool [width*height];
  memcpy (mask, src.mask, width*height);
  return (*this);
}

void RobotMask::writeToStream (std::ostream& out) const throw () {
  out << "P6\n";
  out << width << ' ' << height << '\n';
  out << 255 << '\n';
  for (unsigned int i=0; i<height; i++) {
    for (unsigned int j=0; j<width; j++) {
      unsigned char c = (isValid(j,i) ? 255 : 0);
      out << c << c << c;
    }
  }
  out << flush;
}

RobotMask* RobotMask::dilate () const throw (std::bad_alloc) {
  RobotMask* result = new RobotMask (width, height);
  const bool* ptr = mask;
  bool* rptr = result->mask;
  for (unsigned int j=0; j<height; j++) {
    bool firstrow = (j==0);
    bool lastrow = (j+1==height);
    for (unsigned int i=0; i<width; i++) {
      bool firstcolumn = (i==0);
      bool lastcolumn = (i+1==width);
      (*rptr) = *ptr && (firstcolumn || *(ptr-1)) && (lastcolumn || *(ptr+1)) &&
          (firstrow || (*(ptr-width) && (firstcolumn || *(ptr-width-1)) && (lastcolumn || *(ptr-width+1)))) &&
          (lastrow || (*(ptr+width) && (firstcolumn || *(ptr+width-1))  && (lastcolumn || *(ptr+width+1))));
      rptr++;
      ptr++;
    }
  }
  return result;
}

void RobotMask::addFrame () throw () {
  for (unsigned int i=0; i<width; i++) {
    set (i,0,false);
    set (i,height-1,false);
  }
  for (unsigned int j=0; j<height; j++) {
    set (0,j,false);
    set (width-1,j,false);
  }
}
