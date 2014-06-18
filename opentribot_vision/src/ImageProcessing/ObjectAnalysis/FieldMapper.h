#ifndef _fieldmapper_h_
#define _fieldmapper_h_

#include "../Formation/Image.h"

namespace Tribots {

  /** FieldMapper; teilt das Bild in verschiedene Felder auf und prueft durch 
    Subsampling, in welchen Feldern gruen gefunden wurde
    \attention es bleibt zu pruefen, was der Fieldmapper bei Bildern mit mehr als 640x480 Pixeln macht */
  class FieldMapper {
  public:
    FieldMapper(int x1=0,int x2=640,int y1=0,int y2=480,int w=64,int h=48);
    ~FieldMapper() {;};
 
    /** pruefe, ob Position (x,y) in einem gruenen Feld liegt */
    bool insideField(int x, int y) const;
    /** neues Bild uebergeben */
    void buildFieldMap(const Image& image);
    void drawVisualization(Image* image);
    void growRegions();
  protected:
    int colClass;
    int x1,x2;
    int y1,y2;
    int w,h;
    bool imagemap[64][48];
    int xstep;
    int ystep;
  };

 }

#endif
