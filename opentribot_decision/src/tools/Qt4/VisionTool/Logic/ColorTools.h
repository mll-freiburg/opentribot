#ifndef _COLORTOOLS_H_
#define _COLORTOOLS_H_

#include <string>
#include "../../../../ImageProcessing/ObjectAnalysis/ColorClasses.h"
#include "../../../../ImageProcessing/PixelAnalysis/ColorClassifier.h"
#include "../../../../ImageProcessing/PixelAnalysis/YUVLookupTable.h"

namespace Tribots {
  
  using std::string;
  
  /** Data structure to store thresholds on three color axes to define a
   *  color region in some undefined color space. */
  class ColorRange {
  public:
    ColorRange(double x1min, double x1max,
               double x2min, double x2max,
               double x3min, double x3max,
	       double x4min, double x4max);
    ColorRange();
    
    /** Copy Constructor */
    ColorRange(const ColorRange& cr); 
    
    /** Assignment operator */
    ColorRange& operator=(const ColorRange& cr);
    
    double getMin(int axis) const;
    double getMax(int axis) const;
    
    void setMin(int axis, double v);
    void setMax(int axis, double v);
    
    /** Checks whether or not the given tuple is in the range of this color */
    bool contains(double x1, double x2, double x3, double x4) const;
    /** Checks whether or not the given tuple is in the range of this color. 
     *  Convenience method.*/
    bool contains(double x[4]) const;
    
  protected:
    double mins[4];       ///< holds the minimum thresholds of the three axes
    double maxs[4];       ///< holds the maximum thresholds of the trhee axes
  };
  
  
  class HSISegmentationTool : public ColorClassifier {
  public:
    HSISegmentationTool();
    virtual ~HSISegmentationTool();
    
    
    /** Sets the active color. The image will be segmented with this color 
     *  class. A value < 0 means "all colors". */
    void setActiveColor(int id);
    int getActiveColor() const;
    void setActiveColorAll();
    
    void setColorRange(int id, const ColorRange& cr);
    ColorRange getColorRange(int id) const;
    
    virtual const unsigned char& lookup(const RGBTuple& rgb)  const;
    virtual const unsigned char& lookup(const RGBTuple& rgb, const YUVTuple& yuv)  const;
    virtual const unsigned char& lookup(const YUVTuple&  yuv)  const;
    virtual const unsigned char& lookup(const UYVYTuple& uyvy, int p) const;
    
    virtual void set(const RGBTuple&  rgb,  unsigned char c);
    virtual void set(const YUVTuple&  yuv,  unsigned char c);
    virtual void set(const UYVYTuple& uyvy, unsigned char c, int p);
    
    virtual void load(string);
    virtual void save(string) const;
    
    virtual void createLookupTable();
    virtual void saveLookupTable(string filename) const;
    virtual void loadLookupTable(string filename);
    
    virtual void useLookupTable(bool use=true);
    virtual bool isUseLookupTable() const;
    virtual bool haveLookupTable() const;
    
    virtual ColorClassifier* create() const;
    
  protected:
    int colorID;
    ColorClassInfoList *colorClassInfoList;
    ColorRange *colorRanges;        ///< array of ColorRange objects.
    YUVLookupTable *lookupTable;
    unsigned char cTmp;
    bool useLut;
  };

  /** transforms rgb values to the conical version of the HSL color space.
   *  h = [0,360]   s = [0.,1.]   l = [0.,1.] */
  void rgb2hsi(const RGBTuple& rgb, double &h, double &s, double &i);
  /** transforms rgb values to the cylindric version of the HSI color space.
   *  h = [0,360]   s = [0.,1.]   l = [0.,1.] */
  void rgb2hsl(const RGBTuple& rgb, double &h, double &s, double &v);

  /** conical hsl model with yuv- y channel as intensity 
   *  h = [0,360]   s = [0.,1.]   y = [0.,1.] */
  void rgb2hsy(const RGBTuple& rgb, double &h, double&s, double &y);

  
  // inlines //////////////////////////////////////////////////////////////////
  
  inline double ColorRange::getMin(int axis) const
  {
    return mins[axis];
  }
  inline double ColorRange::getMax(int axis) const
  {
    return maxs[axis];
  }
  
  
  inline void ColorRange::setMin(int axis, double value)
  {
    mins[axis] = value;
  }
  inline void ColorRange::setMax(int axis, double value)
  {
    maxs[axis] = value;
  }
  
  
  inline bool ColorRange::contains(double x[4]) const
  {
    return contains(x[0], x[1], x[2], x[3]);
  }
  
  inline bool ColorRange::contains(double x1, double x2, double x3, double x4) const
  {
    return 
        x1 >= mins[0] && x1 <= maxs[0] &&
        x2 >= mins[1] && x2 <= maxs[1] &&
        x3 >= mins[2] && x3 <= maxs[2] &&
        x4 >= mins[3] && x4 <= maxs[3];
  }
}


#endif
