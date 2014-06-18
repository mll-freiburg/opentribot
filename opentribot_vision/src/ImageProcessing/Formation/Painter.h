#ifndef _PAINTER_H_
#define _PAINTER_H_

#include "Image.h"




namespace Tribots {

class Vec;

  class Painter {
  public:

    static const RGBTuple black;
    static const RGBTuple white;

    enum { PEN_SOLID=0, PEN_DOTTED, PEN_STEPPED };

    Painter(Image& image);

    void setColor(const RGBTuple& color);
    void setColor(int r, int g, int b);

    void setPen(int type);

    void drawLine(int x1, int y1, int x2, int y2);
    void drawLine(const Vec start,const Vec end);
    void drawPoint(int x, int y);
    void drawRect(int x, int y, int width, int height);
    void drawCircle (double x, double y, double radius);
    void drawCircle (Vec center, double radius);
    void drawXYEllipse (Vec center, double radiusX, double radiusY);

    void markCrosshair(int x, int y, int size);   // horizontal, vertical
    void markCrossbar(int x, int y, int size);    // diagonal
    void markRect(int x, int y, int size);
    void drawNumber(int x, int y, int size, int number);

  protected:
    Image& image;
    RGBTuple color;
    int pen;
    int counter;

    void usePen(int x, int y);
  };

}

#endif
