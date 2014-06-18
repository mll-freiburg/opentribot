
#include "DriveVector.h"



std::ostream& operator<< (std::ostream& os, const Tribots::DriveVector& v) {
  os <<"DriveVector "<<v.vtrans.x<<" "<<v.vtrans.y<<" "<<v.vrot<<" "<<v.mode<<" "<<v.vaux[0]<<" "<<v.vaux[1]<<" "<<v.vaux[2]<<" "<<v.kick<<" "<<v.klength <<" ";
  return os;
}

std::istream& operator>> (std::istream &in, Tribots::DriveVector &v)
{
	int mode;
	char name[20];

    in >> name >>v.vtrans.x>>v.vtrans.y >>v.vrot >>mode>>v.vaux[0]>> v.vaux[1]>> v.vaux[2]>> v.kick >> v.klength;

v.mode=(Tribots::DriveVectorMode)mode;
//   std::cout <<name<<std::endl;
    return in;
}

Tribots::DriveVector::DriveVector () throw () {
  vtrans.x=0;
  vtrans.y=0;
  vrot=0;
  mode = ROBOTVELOCITY;
  for (int i=0; i<3; i++) vaux[i]=0; 
  kick	  = NOKICK;
  klength = 180;
}

Tribots::DriveVector::DriveVector (Tribots::Vec vtrans1, double vrot1, unsigned int kick1, unsigned int klength1) throw() 
  : vtrans(vtrans1), vrot(vrot1), kick(kick1), mode(ROBOTVELOCITY), klength(klength1) {;}

Tribots::DriveVector::DriveVector (double v1, double v2, double v3, unsigned int _kick, DriveVectorMode _mode, unsigned int _klength)
{
  klength = _klength;
  kick = _kick;
  mode = _mode;
  switch (mode) {
  case ROBOTVELOCITY:
    vtrans.x = v1;
    vtrans.y = v2;
    vrot     = v3;
    for (int i=0; i<3; i++) vaux[i] = 0;
    break;
  case WHEELVELOCITY:
  case MOTORVOLTAGE:
    vaux[0]=v1;
    vaux[1]=v2;
    vaux[2]=v3;
    vtrans.x = 0;
    vtrans.y = 0;
    vrot     = 0;
    break;
  }
}

Tribots::DriveVector::DriveVector (const Tribots::DriveVector& src) throw () 
    : vtrans(src.vtrans), vrot(src.vrot), kick(src.kick), mode(src.mode), klength(src.klength)
{
    for (int i=0; i<3; i++) vaux[i]=src.vaux[i];
}

const Tribots::DriveVector& Tribots::DriveVector::operator= (const Tribots::DriveVector& src) throw () {
  vtrans=src.vtrans;
  vrot = src.vrot;
  kick = src.kick;
  klength = src.klength;
  mode = src.mode;
  for (int i=0; i<3; i++) vaux[i]=src.vaux[i];
  return (*this);
}
