
#include "Convolution.h"

using namespace std;
using namespace Tribots;

Convolution::Convolution () throw () : dimX(0), dimY(0), offset(0) {;}

unsigned int Convolution::boundarySizeX() const throw () {
  return dimX/2;
}

unsigned int Convolution::boundarySizeY() const throw () {
  return dimY/2;
}

void Convolution::operator () (GrayLevelImage& dest, const GrayLevelImage& src) throw () {
  int w = src.width();
  int h = src.height();
  int offX = dimX/2;
  int offY = dimY/2;
  for (int y=offY; y+offY<h; y++) {
    for (int x=offX; x+offX<w; x++) {
      float sum=offset;
      vector<float>::iterator it = mask.begin();
      for (int j=-offY; j<=offY; j++) {
        for (int i=-offX; i<=offX; i++) {
          sum+=(*(it++))*src(x+i,y+j);
        }
      }
      dest(x,y)=sum;
    }
  }
}

GaussianSmoothing::GaussianSmoothing (unsigned int width) throw () {
  mask.resize (width*width);
  dimX=dimY=width;
  if (width<3)
    width=3;
  if (width%2==0)
    width++;
  // Pascalsches Dreieck aufbauen:
  vector<float> cvm;
  cvm.assign (width, 1.0);
  cvm[0]=1;
  for (unsigned int i=3; i<=width; i++) {
    for (unsigned int j=i-2; j>0; j--) {
      cvm[j]+=cvm[j-1];
    }
  }
  float sum=0;
  for (unsigned int i=0; i<width; i++) {
    sum+=cvm[i];
  }
  for (unsigned int i=0; i<width; i++) {
    for (unsigned int j=0; j<width; j++) {
      mask[width*i+j]=cvm[i]*cvm[j]/(sum*sum);
    }
  }
}

SobelX::SobelX () throw () {
  dimX=dimY=3;
  offset=0;
  mask.resize (9);
  mask[0]=mask[6]=-1;
  mask[2]=mask[8]=+1;
  mask[3]=-2;
  mask[5]=+2;
  mask[1]=mask[4]=mask[7]=0;
}

SobelY::SobelY () throw () {
  dimX=dimY=3;
  offset=0;
  mask.resize (9);
  mask[0]=mask[2]=-1;
  mask[6]=mask[8]=+1;
  mask[1]=-2;
  mask[7]=+2;
  mask[3]=mask[4]=mask[5]=0;
}
