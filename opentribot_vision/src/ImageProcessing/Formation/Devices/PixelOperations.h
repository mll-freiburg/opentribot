#ifndef PIXELOPERATIONS_H_
#define PIXELOPERATIONS_H_

class PixelOperations
{
public:
	PixelOperations();
	virtual ~PixelOperations();


// types of pixels

struct yuv422pkt
{
  unsigned char u;
  unsigned char y1;
  unsigned char v;
  unsigned char y2;
};

struct rgb2pkt
{
        unsigned char r1;
        unsigned char g1;
        unsigned char b1;
        unsigned char r2;
        unsigned char g2;
        unsigned char b2;

};

struct rgbpkt
{
        unsigned char r;
        unsigned char g;
        unsigned char b;

};

};


#endif /*PIXELOPERATIONS_H_*/
