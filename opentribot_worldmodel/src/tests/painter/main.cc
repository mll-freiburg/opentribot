#include <fstream>
#include <iostream>

#include "../../ImageProcessing/Painter.h"
#include "../../ImageProcessing/Images/RGBImage.h"
#include "../../ImageProcessing/JPEGIO.h"

using namespace std;
using namespace Tribots;

int main(int argc, char* argv[])
{
  if (argc!=2) {
    cerr << "Usage: " << argv[0] << " output_filename" << endl;
    exit(1);
  }
  ofstream out(argv[1]);
  if (!out) {
    cerr << "Could not open file " << argv[1] << " for writing." << endl;
    exit(1);
  }
  
  Image* image = new RGBImage(320, 240);
  Painter p(*image);
  
  p.setColor(p.white);
  p.drawLine(0,0,320,240);
  
  p.setPen(Painter::PEN_DOTTED);
  p.drawLine(320,0,0,240);
  
  p.setPen(Painter::PEN_STEPPED);
  p.drawLine(68, 23, 299, 188);

  p.setColor(200, 0, 0);
  p.setPen(Painter::PEN_SOLID);
  p.drawLine(299, 23, 68, 188);

  ImageIO* io = new PPMIO();
  io->write(*image, out);

  out.close();
  return 0;
}
