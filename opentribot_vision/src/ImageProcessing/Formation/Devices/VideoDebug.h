#ifndef VIDEODEBUG_H_
#define VIDEODEBUG_H_

#include "VideoUtils.h"

#include "PixelOperations.h"

#include "smallcvtk/PixelConversion.h"


#define PIXEL_YUV2RGB(y,u,v, r,g,b)                        \
	((r) = (y)+(((v)*1436)>>10),                         \
			(g) = (y)-(((u)*352+(v)*731)>>10),                  \
			(b) = (y)+(((u)*1814)>>10))


#define PIXEL_RGB2YUV(r, g, b, y, u, v)                    \
	((y) = (306*(r) + 601*(g) + 117*(b))  >> 10,         \
			(u) = ((-172*(r) - 340*(g) + 512*(b)) >> 10) + 128, \
			(v) = ((512*(r) - 429*(g) - 83*(b)) >> 10) + 128)

class VideoDebug
{
public:

	VideoDebug();
	virtual ~VideoDebug();

	static unsigned char* YUV420_to_RGB(const unsigned char* yuv420,int width, int height);

	static void writePPM(unsigned char* rgbbuf, int width, int height, const char* filename);

	static void writePPM_YUV_Half(const unsigned char* yuvbuf, int width, int height, const char* filename);

	static void loadPPM_YUV_Half(unsigned char* yuvbuf, int width, int height, const char* filename);

	static void loadJPG_YUV_Half(unsigned char* yuvbuf, int width, int height, const char* filename);

};

class Painter
{

public:
	int x,y;
	int height,width;
	int r,g,b;
	int color;
	unsigned char*image;
	int init(unsigned char* _image,int _width,int _height,int r,int g,int b){
		this->setcolor(r,g,b);
		image=_image;
		height=_height;
		width=_width;
		return 0;
	}
	int setcolor(int _r,int _g,int _b){
		//color=_r*255*255+_g*255+_b;
		r=_r;g=_g;b=_b;
		return 0;
	}
	int go(int tox,int toy,bool paint){

		if (paint==0){
			x=tox;y=toy;
			return 0;
		}
		else{
			line(x,tox,y,toy);
			x=tox;y=toy;
		}

	}

	int rect(int w,int h){
		w=w/2;
		h=h/2;
		line(x-w,x+w,y-h,y-h);
		line(x+w,x+w,y-h,y+h);
		line(x+w,x-w,y+h,y+h);
		line(x-w,x-w,y+h,y-h);


		return 0;
	}

	int cross(int size){

		line(x-size,x+size,y-size,y+size);
		line(x+size,x-size,y-size,y+size);


		return 0;
	}



	int plot(int x,int y){
		unsigned char* p=&image[width*y*3+x*3];

		*p=r;
		*(p+1)=g;
		*(p+2)=b;
		return 0;
	}


	int line(int x0, int x1, int y0, int y1){

		if (x0<0)x0=0;
		if (x1<0)x1=0;
		if (y0<0)y0=0;
		if (y1<0)y1=0;
		if (x0>width-1)x0=width-1;
		if (x1>width-1)x1=width-1;
		if (y0>height-1)y0=height-1;
		if (y1>height-1)y1=height-1;



		bool steep = abs(y1 - y0) > abs(x1 - x0);
		int s;
		if (steep){
			s=x0;x0=y0;y0=s;
			s=x1;x1=y1;y1=s;
		}
		if (x0 > x1){
			s=x0;x0=x1;x1=s;
			s=y0;y0=y1;y1=s;
		}
		int deltax = x1 - x0;
		int deltay = abs(y1 - y0);
		int error = -(deltax + 1) / 2;
		int ystep;
		int _y = y0;
		int _x;
		if (y0 < y1)ystep = 1; else ystep = -1;
		for (_x=x0;_x<=x1;_x++){
			if (steep) plot(_y,_x); else plot(_x,_y);
			error = error + deltay;
			if (error >= 0){
				_y = _y + ystep;
				error = error - deltax;
			}
		}

		return 0;
	}



};


#endif /*VIDEODEBUG_H_*/
