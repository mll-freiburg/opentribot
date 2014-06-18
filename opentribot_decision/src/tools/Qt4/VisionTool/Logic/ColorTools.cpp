#include "ColorTools.h"
#include <fstream>
#include "../../../../ImageProcessing/Formation/PixelConversion.h"
#include "../../../../Structures/TribotsException.h"

using namespace std;

namespace Tribots {
  
  ColorRange::ColorRange(double x1min, double x1max,
                         double x2min, double x2max,
                         double x3min, double x3max,
			 double x4min, double x4max)
  {
    mins[0]=x1min; maxs[0]=x1max;
    mins[1]=x2min; maxs[1]=x2max;
    mins[2]=x3min; maxs[2]=x3max;
    mins[3]=x4min; maxs[3]=x4max;
  }
  
  ColorRange::ColorRange()
  {
    mins[0]=mins[1]=mins[2]=maxs[0]=maxs[1]=maxs[2]=maxs[3]=mins[3]=0;
  }
  
  ColorRange& ColorRange::operator=(const ColorRange& cr)
                                   {
    for (int i=0; i < 4; i++) {
      this->mins[i] = cr.mins[i];
      this->maxs[i] = cr.maxs[i];
    }
    return *this;
  }
  
  
  ColorRange::ColorRange(const ColorRange& cr)
  {
    for (int i=0; i < 4; i++) {
      this->mins[i] = cr.mins[i];
      this->maxs[i] = cr.maxs[i];
    }
  }
  
  HSISegmentationTool::HSISegmentationTool()
    : colorID(0), colorClassInfoList(new ColorClassInfoList()),
      lookupTable(0), useLut(false)
  {
    colorRanges = new ColorRange[colorClassInfoList->classList.size()];
  }
  HSISegmentationTool::~HSISegmentationTool()
  {
    delete [] colorRanges;
    delete colorClassInfoList;
  }
  
  void HSISegmentationTool::setActiveColor(int id)
  {
    colorID = id;
  }
  int HSISegmentationTool::getActiveColor() const
  {
    return colorID;
  }
  void HSISegmentationTool::setActiveColorAll()
  {
    colorID = 0;
  }
  
  void HSISegmentationTool::setColorRange(int id, const ColorRange& cr)
  {
    colorRanges[id] = cr;
    /* // print out errors
    cerr << "ColorRange: " << cr.getMin(0) << "-" << cr.getMax(0) << " "
         << cr.getMin(1) << "-" << cr.getMax(1) << " "
         << cr.getMin(2) << "-" << cr.getMax(2) << endl;
         */
  }
  ColorRange HSISegmentationTool::getColorRange(int id) const
  {
    return colorRanges[id];
  }

  const unsigned char&HSISegmentationTool::lookup(const RGBTuple& rgb) const
  {
    YUVTuple yuvTmp;
    PixelConversion::convert(rgb, &yuvTmp);
    return lookup(rgb, yuvTmp);    
  }
  
  const unsigned char& HSISegmentationTool::lookup(const RGBTuple& rgb, 
						   const YUVTuple& yuv) const
  {
    if (useLut && lookupTable) {
      return lookupTable->lookup(rgb);
    }     
    
    double h,s,i;
    int b = ((int)yuv.v) - ((int)yuv.u) - ((int)yuv.y)/2;
    rgb2hsy(rgb, h, s, i);// use rgb2hsl transformation
    
    unsigned char c = 0;
    if (colorID != 0) {
      c =                 // farbe oder hintergrund ?
	((colorRanges[colorID].getMin(0) <= h && 
	  colorRanges[colorID].getMax(0) >= h)   ||
	 (colorRanges[colorID].getMin(0) > colorRanges[colorID].getMax(0) &&
	  (colorRanges[colorID].getMin(0) <= h || 
	   colorRanges[colorID].getMax(0) >= h)))     &&
	(colorRanges[colorID].getMin(1) <= s && 
	 colorRanges[colorID].getMax(1) >= s)      &&
	(colorRanges[colorID].getMin(2) <= i && 
	 colorRanges[colorID].getMax(2) >= i) &&
	(colorRanges[colorID].getMin(3) <= b &&
	 colorRanges[colorID].getMax(3) >= b)   ? colorID : 0 ; 
    }
    else {  // alle Farben
      for (unsigned int id = 1; 
	   id < colorClassInfoList->classList.size(); id++) {
        if (((colorRanges[id].getMin(0) <= h && 
	      colorRanges[id].getMax(0) >= h)   ||
	     (colorRanges[id].getMin(0) > 
	      colorRanges[id].getMax(0) &&
	      (colorRanges[id].getMin(0) <= h || 
	       colorRanges[id].getMax(0) >= h)))     &&
	    (colorRanges[id].getMin(1) <= s && 
	     colorRanges[id].getMax(1) >= s)     &&
	    (colorRanges[id].getMin(2) <= i && 
	     colorRanges[id].getMax(2) >= i) &&
	    (colorRanges[id].getMin(3) <= b &&
	     colorRanges[id].getMax(3) >= b)) {
	  c = id;
          break;
	}
      }
    }
    const_cast<HSISegmentationTool*>(this)->cTmp = c; // unschoen, noch mal ran
    return cTmp;
  }
  
  const unsigned char& HSISegmentationTool::lookup(const YUVTuple& yuv) const
  {
    if (useLut && lookupTable) {
      return lookupTable->lookup(yuv);
    }    
    RGBTuple rgbTmp;
    PixelConversion::convert(yuv, &rgbTmp);
    return lookup(rgbTmp, yuv);
  }
  
  const unsigned char& HSISegmentationTool::lookup(const UYVYTuple& uyvy, int) const  // hack, ignores p
  {
    if (useLut && lookupTable) {
      return lookupTable->lookup(uyvy);
    }
    RGBTuple rgbTmp; YUVTuple yuvTmp;
    PixelConversion::convert(uyvy, &rgbTmp);
    PixelConversion::convert(uyvy, &yuvTmp);
    return lookup(rgbTmp, yuvTmp);
  }
  
  void HSISegmentationTool::useLookupTable(bool use)
  {
    useLut = use;
  }
  bool HSISegmentationTool::isUseLookupTable() const
  {
    return useLut;
  }  
  bool HSISegmentationTool::haveLookupTable() const
  {
    return lookupTable != 0;
  }  
  
  void HSISegmentationTool::createLookupTable()
  {
    bool tmp = useLut;
    useLut = false;
    if (! lookupTable) {
      lookupTable = new YUVLookupTable();
    }
    int c = getActiveColor();
    setActiveColorAll();
    lookupTable->fillFromClassifier(this);
    setActiveColor(c);
    useLut = tmp;
  }
    
  void HSISegmentationTool::saveLookupTable(string filename) const
  {
    if (! lookupTable) {
      const_cast<HSISegmentationTool*>(this)->createLookupTable();
    }
    lookupTable->save(filename);
  }
  
  void HSISegmentationTool::loadLookupTable(string filename)
  {
    if (! lookupTable) {
      lookupTable = new YUVLookupTable();
    }
    lookupTable->load(filename);
  }
  
  void HSISegmentationTool::set(const RGBTuple&, unsigned char)
  { return; }
  void HSISegmentationTool::set(const YUVTuple&, unsigned char)
  { return; }
  void HSISegmentationTool::set(const UYVYTuple&, unsigned char, int)
  { return; }
  
  void HSISegmentationTool::load(string filename)
  {
    ifstream in(filename.c_str());
    if (!in) {
      throw TribotsException("Could not read file.");
    }
    for (unsigned int i=1; i < colorClassInfoList->classList.size(); i++) {
      double hMin, hMax, sMin, sMax, iMin, iMax, bMin, bMax;
      in >> hMin >> hMax >> sMin >> sMax >> iMin >> iMax >> bMin >> bMax;
      colorRanges[i].setMin(0, hMin);
      colorRanges[i].setMax(0, hMax);
      colorRanges[i].setMin(1, sMin);
      colorRanges[i].setMax(1, sMax);
      colorRanges[i].setMin(2, iMin);
      colorRanges[i].setMax(2, iMax);
      colorRanges[i].setMin(3, bMin);
      colorRanges[i].setMax(3, bMax);
    }

    in.close();
  }
  
  void HSISegmentationTool::save(string filename) const
  {
    ofstream out(filename.c_str());
    if (! out ) {
      throw TribotsException("Could not write file.");       
    }
    
    for (unsigned int i=1; i < colorClassInfoList->classList.size(); i++) {
      out << colorRanges[i].getMin(0) << " " 
          << colorRanges[i].getMax(0) << " "
          << colorRanges[i].getMin(1) << " " 
          << colorRanges[i].getMax(1) << " "
          << colorRanges[i].getMin(2) << " " 
          << colorRanges[i].getMax(2) << " " 
          << colorRanges[i].getMin(3) << " " 
          << colorRanges[i].getMax(3) << endl;
    }    
    out.close();
  }
  
  ColorClassifier* HSISegmentationTool::create() const
  {
    return new HSISegmentationTool();
  }


  /* conical hsl model with yuv- y channel as intensity */
  void rgb2hsy(const RGBTuple& rgb, double &h, double&s, double &y)
  {
    double min, max, delta;
    double r = rgb.r / 255.0;
    double g = rgb.g / 255.0;
    double b = rgb.b / 255.0;

    YUVTuple yuv;

    PixelConversion pc;
    pc.convert(rgb, &yuv);
    
    min = (r<g) ? r : g;
    min = min < b ? min : b;
    max = (r>g) ? r : g;
    max = max > b ? max : b;
    delta = max - min;
    
    y = yuv.y / 255.;
    s = delta;
    
    if (delta != 0.) {
      if (r == max) h = (g-b) / delta;
      else if (g == max) h = 2.0 + (b-r)/delta;
      else h = 4.0+(r-g)/delta;
    }
    else h = 0.;
    
    if ((h*=60.) < 0.) h += 360.;
  }


  /* conical hsl model */
  void rgb2hsl(const RGBTuple& rgb, double &h, double&s, double &l)
  {
    double min, max, delta;
    double r = rgb.r / 255.0;
    double g = rgb.g / 255.0;
    double b = rgb.b / 255.0;
    
    min = (r<g) ? r : g;
    min = min < b ? min : b;
    max = (r>g) ? r : g;
    max = max > b ? max : b;
    delta = max - min;
    
    l = (max + min) / 2;
    s = delta;
    
    if (delta != 0.) {
      if (r == max) h = (g-b) / delta;
      else if (g == max) h = 2.0 + (b-r)/delta;
      else h = 4.0+(r-g)/delta;
    }
    else h = 0.;
    
    if ((h*=60.) < 0.) h += 360.;
  }

  //  Hilfsfunktion zur Berechnung der Transformation von rgb nach hsi.
  void rgb2hsi(const RGBTuple& rgb, double &h, double &s, double &i)
  {   // r,g,b = [0,255]
    // h = [0,360], s = [0,1], v = [0,1]
    // for black, grey and white, angle 0 is selected as color...
    
    double min, max, delta;
    
    double r = rgb.r / 255.0;
    double g = rgb.g / 255.0;
    double b = rgb.b / 255.0;
    
    min = (r<g) ? r : g;
    if (min > b) min = b;
    max = (r>g) ? r : g;
    if (max < b) max = b;
    i = max;
    delta = max - min;
    
    if(max != 0.) s = delta / max;
    else {                                      // r = g = b = 0  
      s = 0.;                                   // s = 0, v is undefined
      h = 0.; // h = -1;    // no color (black)
      return;
    }
    
    if (delta != 0.) {
      if (r == max) h = (g-b)/delta;             // between yellow & magenta
      else if (g == max) h = 2.0+(b-r)/delta;    // between cyan & yellow
      else  h = 4.0+(r-g)/delta;                 // between magenta & cyan
    }
    else {                                       // white, grey, black
      h = 0.;
      return;
    }
    
    h *= 60.0;                  // degrees
    if(h < 0.0) h += 360.0;
    
    /** // print out transformation errors
    if (h <0 || h > 360 || s < 0 || s > 1. || i < 0 || i > 1.) {
      cerr << "HSI: " << h << " " << s << " " << i << "   RGB: "
           << r << " " << g << " " << b << endl;
    }  */
  }
  

}
