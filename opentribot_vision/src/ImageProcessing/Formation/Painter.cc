#include "Painter.h"
#include <cmath>
#include "../../Fundamental/Vec.h"
#include <stdlib.h>

namespace Tribots {

  Painter::Painter(Image& image) 
    : image(image), color(black), pen(PEN_SOLID), counter(0) 
  {}
  void Painter::setColor(const RGBTuple& color)
  {
    this->color = color;
  }
  void Painter::setColor(int r, int g, int b)
  {
    color.r = r;
    color.g = g;
    color.b = b;
  }

  void Painter::setPen(int type)
  {
    pen = type;
  }

  void Painter::usePen(int x, int y)
  {
    switch (pen) {
    case PEN_DOTTED: 
      if (counter++ % 2 == 0) {
        image.setPixelRGB(x,y,color);
      }
      break;
    case PEN_STEPPED:
      if (counter++ % 7 < 4) {
        image.setPixelRGB(x,y,color);
      }
      break;
    case PEN_SOLID: 
    default: 
      image.setPixelRGB(x,y,color); break;
    }
  }

  void Painter::drawRect(int x, int y, int width, int height)
  {
    drawLine(x,y, x+width, y);
    drawLine(x+width, y, x+width, y+height);
    drawLine(x+width, y+height, x, y+height);
    drawLine(x, y+height, x, y);
  }

  void Painter::markRect(int x, int y, int size)
  {
    drawRect(x-size, y-size, size*2, size*2);
  }

  void Painter::markCrosshair(int x, int y, int size)
  {
    drawLine(x, y-size, x, y+size);
    drawLine(x-size, y, x+size, y);
  }

  void Painter::markCrossbar(int x, int y, int size)
  {
    drawLine(x-size, y-size, x+size, y+size);
    drawLine(x+size, y-size, x-size, y+size);
  }

  void Painter::drawPoint(int x, int y)
  {
    image.setPixelRGB(x, y, color);
  }

  // Implementierung des Bresenham-Algorithmus mit Integern
  void Painter::drawLine(const Vec start,const Vec end)
  {
    drawLine((int)start.x,(int)start.y,(int)end.x,(int)end.y);
  }
  // Implementierung des Bresenham-Algorithmus mit Integern
  void Painter::drawLine(int x1, int y1, int x2, int y2)
  {
    int x=x1,y=y1, error, delta, schwelle, dx, dy, inc_x, inc_y;
    int width = image.getWidth();
    int height= image.getHeight();

    dx = x2-x;
    dy = y2-y;

    if (dx>0) inc_x=1; else inc_x=-1;
    if (dy>0) inc_y=1; else inc_y=-1;
    if (abs(dy) < abs(dx)) {
      error = -abs(dx);
      delta = 2* abs(dy);
      schwelle = 2*error;
      while (x != x2) {
        if (y<0 || x<0 || y>=height || x>=width)
          ;             // don't draw pixels, that aren't in the visual's area
        else
          usePen(x,y);
        x+=inc_x;
        error+=delta;
        if (error>0) { y+=inc_y; error+=schwelle; }
      }
    }
    else {
      error = -abs(dy);
      delta = 2* abs(dx);
      schwelle = 2*error;
      while (y != y2) {
        if (y<0 || x<0 || y>=height || x>=width)
          ;
        else
          usePen(x,y);
        y+=inc_y;
        error+=delta;
        if (error>0) { x+=inc_x; error+=schwelle; }
      }
    }

    if (y<0 || x<0 || y>=height || x>=width)
      return;

    usePen(x,y);
  }

  void Painter::drawCircle(Vec center, double radius) {
    drawCircle(center.x,center.y,radius);
  }
  void Painter::drawCircle(double x, double y, double radius) {
    int xp = static_cast<int>(x+radius);
    int yp = static_cast<int>(y);
    for (int i=0; i<=64; i++) {
      double a = (i*2.0*M_PI)/64.0;
      int xn = static_cast<int>(x+radius*std::cos(a));
      int yn = static_cast<int>(y+radius*std::sin(a));
      drawLine (xp, yp, xn, yn);
      xp=xn;
      yp=yn;
    }
  }
  void Painter::drawXYEllipse(Vec center, double radiusX, double radiusY) {
    int xp = static_cast<int>(center.x+radiusX);
    int yp = static_cast<int>(center.y);
    for (int i=0; i<=64; i++) {
      double a = (i*2.0*M_PI)/64.0;
      int xn = static_cast<int>(center.x+radiusX*std::cos(a));
      int yn = static_cast<int>(center.y+radiusY*std::sin(a));
      drawLine (xp, yp, xn, yn);
      xp=xn;
      yp=yn;
    }
  }

  const RGBTuple Painter::black = {0,0,0};
  const RGBTuple Painter::white = {255,255,255};

  void Painter::drawNumber(int x, int y, int size, int number) {
    int kl=size/2;
    if (number<0) {
      drawLine (x,y,x+kl,y);
      x+=7*size/6;
      number*=-1;
    }
    int divisor = static_cast<int>(pow(10, floor(log(number)/log(10))));
    while (divisor>0) {
      int c = number/divisor;
      bool segments [7];
      segments[0]=segments[1]=segments[2]=segments[3]=segments[4]=segments[5]=segments[6]=false;
      switch (c) {
        case 0:
          segments[0]=segments[1]=segments[2]=segments[3]=segments[4]=segments[5]=true;
          break;
        case 1:
          segments[3]=segments[4]=true;
          break;
        case 2:
          segments[1]=segments[2]=segments[4]=segments[5]=segments[6]=true;
          break;
        case 3:
          segments[2]=segments[3]=segments[4]=segments[5]=segments[6]=true;
          break;
        case 4:
          segments[0]=segments[3]=segments[4]=segments[6]=true;
          break;
        case 5:
          segments[0]=segments[2]=segments[3]=segments[5]=segments[6]=true;
          break;
        case 6:
          segments[0]=segments[1]=segments[2]=segments[3]=segments[5]=segments[6]=true;
          break;
        case 7:
          segments[3]=segments[4]=segments[5]=true;
          break;
        case 8:
          segments[0]=segments[1]=segments[2]=segments[3]=segments[4]=segments[5]=segments[6]=true;
          break;
        case 9:
          segments[0]=segments[2]=segments[3]=segments[4]=segments[5]=segments[6]=true;
          break;
        default:
          break;
      }
      if (segments[0])
        drawLine (x,y,x,y-kl);
      if (segments[1])
        drawLine (x,y,x,y+kl);
      if (segments[2])
        drawLine (x,y+kl,x+kl,y+kl);
      if (segments[3])
        drawLine (x+kl,y,x+kl,y+kl);
      if (segments[4])
        drawLine (x+kl,y,x+kl,y-kl);
      if (segments[5])
        drawLine (x,y-kl,x+kl,y-kl);
      if (segments[6])
        drawLine (x,y,x+kl,y);
      divisor/=10;
      number%=10;
      x+=7*size/6;
    }
  }
}
