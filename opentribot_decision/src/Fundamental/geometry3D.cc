
#include "geometry3D.h"
#include <cmath>

using namespace Tribots;
using namespace std;

namespace {

  // berechne das Teilverhaeltnis von p bzgl (v1,v2) unter der Annahme, dass v1,v2,p auf einer Geraden liegen und v1!=v2
  inline double teilverhaeltnis (const Vec3D& v1, const Vec3D& v2, const Vec3D& p) {
    if (v1.x!=v2.x)
      return (p.x-v1.x)/(v2.x-v1.x);
    else if(v1.y!=v2.y)
      return (p.y-v1.y)/(v2.y-v1.y);
    else 
      return (p.z-v1.z)/(v2.z-v1.z);
  }

}


std::ostream& operator<< (std::ostream& os, const Line3D& v) {
  os << v.p1 << " --> " << v.p2;
  return os;
}

Tribots::Line3D::Line3D (const Tribots::Vec3D pp1, const Tribots::Vec3D pp2) throw (std::invalid_argument)
                        : p1(pp1), p2(pp2) {
  if (pp1==pp2)
    throw std::invalid_argument ("invalid 3D-line definition in Tribots::Line3D::Line3D");
}

//Tribots::Line3D::Line3D (const Tribots::Line& ln) throw () : p1(Vec3D(ln.p1)), p2(Vec3D(ln.p2)) {;}

Tribots::Line3D::Line3D (const Tribots::Line3D& ln) throw () : p1(ln.p1), p2(ln.p2) {;}

Tribots::Line3D::Line3D () throw () : p1(Vec3D(0.,0.,0.)), p2(Vec3D(0.,0.,0.))
{;}

const Tribots::Line3D& Tribots::Line3D::operator= (const Tribots::Line3D& ln) throw () {
  p1=ln.p1;
  p2=ln.p2;
  return (*this);
}

Vec3D
Line3D::intersectZPlane(const double z) const{
   double s = (z-p1.z)/(p2.z-p1.z);
   return p1+s*(p2-p1);
}

Vec3D
Line3D::intersectYPlane(const double y) const{
   double s = (y-p1.y)/(p2.y-p1.y);
   return p1+s*(p2-p1);
}


double Tribots::Line3D::distance (const Tribots::Vec3D p) throw () {
  Vec3D d = p2-p1;
  Vec3D l=  p1+((p*d-p1*d)/(d.squared_length()))*d;
  return (p-l).length();
}


Tribots::Line3D&
Tribots::Line3D::s_rotate (const Tribots::Angle& a) throw () 
{
  p1=p1.rotate(a); p2=p2.rotate(a);  
  return *this;
}

Tribots::Line3D&
Tribots::Line3D::s_translate (const Tribots::Vec3D& v) throw () 
{
  p1+=v; p2+=v;                      
  return *this;
}



Tribots::LineSegment3D&
Tribots::LineSegment3D::s_rotate (const Tribots::Angle& a) throw () 
{
  p1=p1.rotate(a); p2=p2.rotate(a); 
  return *this;
}

Tribots::LineSegment3D&
Tribots::LineSegment3D::s_translate (const Tribots::Vec3D& v) throw () 
{
  p1+=v; p2+=v;                     
  return *this;
}

Line3D Line3D::rotate (const Angle a) const throw () {
  Line3D dest (*this);
  dest.s_rotate (a);
  return dest;
}

Line3D Line3D::translate (const Vec3D v) const throw () {
  Line3D dest (*this);
  dest.s_translate (v);
  return dest;
}

Tribots::Vec3D Tribots::intersectSkewLines(Tribots::Line3D& l1, Tribots::Line3D& l2)
{
  return Tribots::intersectSkewLines(l1.p1,l1.p2-l1.p1,l2.p1,l2.p2-l2.p1);
}

Tribots::Vec3D Tribots::intersectSkewLines(Tribots::Vec3D L1Start, Tribots::Vec3D L1Rel, Tribots::Vec3D L2Start, Tribots::Vec3D L2Rel)
{
  // see http://fermi.jhuapl.edu/s1r/idl/s1rlib/vectors/v_skew.html

  // U1 = (B-A)/| B-A |
  Tribots::Vec3D U1 = L1Rel.normalize();
  // U2 = (D-C)/| D-C |
  Tribots::Vec3D U2 = L2Rel.normalize();
  // U3 = (U1 cross U2)/| U1 cross U2 |
  Tribots::Vec3D U3 = U1.crossProduct(U2);
  U3 = U3.normalize();

  // E = ((A-C) dot U3) U3
  Tribots::Vec3D E = ((L1Start-L2Start) * U3) * U3;

  // 'virtual L1' line which is coplanar with L2
  // (we only need starting point, since line practically has no end)
  Tribots::Vec3D CDash = L2Start + E;
  
  // Unit vector U4 perpendicular to Line 2': 
  //  V = A - C' 
  // V_perp = V - (V dot U2) U2 
  // U4 = V_perp/| V_perp |
  Tribots::Vec3D V = L1Start - CDash;
  Tribots::Vec3D U4 = V - ((V * U2) * U2);
  U4 = U4.normalize();
  
  Tribots::Vec3D IDash = Tribots::intersect(Tribots::Line3D(L1Start,L1Start+L1Rel),Tribots::Line3D(CDash,CDash+L2Rel));
  Tribots::Vec3D I = IDash - (E / 2.0);

  return(I);
}

Tribots::Vec3D Tribots::intersect (const Tribots::Line3D& ln1, const Tribots::Line3D& ln2)
    throw (std::invalid_argument) {

  // Adapted from Mathworld, http://mathworld.wolfram.com/Line-LineIntersection.html
  
  Tribots::Vec3D a = ln1.p2 - ln1.p1;
  Tribots::Vec3D b = ln2.p2 - ln2.p1;
  Tribots::Vec3D c = ln2.p1 - ln1.p1;  
  Tribots::Vec3D x = ln1.p1 + a*((c.crossProduct(b))*(a.crossProduct(b))/a.crossProduct(b).squared_length());
  

//  if(x.x !=nan && x.y != nan&& x.z != nan)
  return x;
//  else
//     throw std::invalid_argument("3Dlines do not intersect in Tribots::intersect");
    
}


Tribots::Vec3D Tribots::Line3D::perpendicular_point (const Tribots::Vec3D& p) throw () {
  Vec3D d=p2-p1;
  return p1+((p*d-p1*d)/(d.squared_length()))*d;
}



/*

Tribots::LineSegment::LineSegment (const Tribots::Vec start, const Tribots::Vec ende) throw (std::invalid_argument) : p1 (start), p2 (ende) {;}

Tribots::LineSegment::LineSegment (const Tribots::LineSegment& ls) throw () : p1(ls.p1), p2(ls.p2) {;}

const Tribots::LineSegment& Tribots::LineSegment::operator= (const Tribots::LineSegment& ls) throw () {
  p1=ls.p1;
  p2=ls.p2;
  return (*this);
}

double Tribots::LineSegment::distance (const Vec p) throw () {
  Vec f = Line (p1, p2).perpendicular_point (p);
  double tv;
  if (p1.x!=p2.x)
    tv = (f.x-p1.x)/(p2.x-p1.x);
  else
    tv = (f.y-p1.y)/(p2.y-p1.y);
  if (tv>=1)
    return (p2-p).length();
  else if (tv<=0)
    return (p1-p).length();
  else
    return (f-p).length();
}

const Tribots::Vec&
Tribots::LineSegment::getStart() const throw ()
{ return p1; }

const Tribots::Vec&
Tribots::LineSegment::getEnd() const throw ()
{ return p2; }


Arc::Arc (Vec c, double r, Angle s, Angle e) throw () : center(c), radius(r), start(s), end(e) {;}

Arc::Arc (const Arc& a) throw () : center(a.center), radius(a.radius), start(a.start), end(a.end) {;}

const Arc& Arc::operator= (const Arc& a) throw () {
  center=a.center;
  radius=a.radius;
  start=a.start;
  end=a.end;
  return (*this);
}

double Arc::distance (const Vec p) throw () {
  Vec v = p-center;
  if (v.angle().in_between (start, end))
    return abs(v.length()-radius);
  else {
    double d1 = (Vec::unit_vector (start)-p).squared_length();
    double d2 = (Vec::unit_vector (end)-p).squared_length();
    if (d1<d2)
      return sqrt(d1);
    else
      return sqrt(d2);
  }
}

std::vector<Vec> Tribots::intersect (const Line& l, const Arc& a) throw (std::bad_alloc) {
  std::vector<Vec> res = intersect (l, Circle (a.center, a.radius));
  unsigned int i=0;
  while (i<res.size())
    if (!(res[i]-a.center).angle().in_between (a.start, a.end))
      res.erase (res.begin()+i);
    else
      i++;
  return res;
}

std::vector<Vec> Tribots::intersect (const Arc& a, const Line& l) throw (std::bad_alloc) {
  return intersect (l,a);
}

std::vector<Vec> Tribots::intersect (const LineSegment& l, const Arc& a) throw (std::bad_alloc) {
  std::vector<Vec> res = intersect (Line (l.p1, l.p2), a);
  unsigned int i=0;
  while (i<res.size()) {
    double tv;
    if (l.p1.x!=l.p2.x)
      tv = (res[i].x-l.p1.x)/(l.p2.x-l.p1.x);
    else
      tv = (res[i].y-l.p1.y)/(l.p2.y-l.p1.y);
    if (tv>1 || tv<0)
      res.erase (res.begin()+i);
    else
      i++;
  }
  return res;
}

std::vector<Vec> Tribots::intersect (const Arc& a, const LineSegment& l) throw (std::bad_alloc) {
  return intersect (l,a);
}
    
std::vector<Vec> Tribots::intersect (const LineSegment& l1, const Line& l2) throw (std::bad_alloc) {
  Vec is;
  try{
    is = intersect (Line (l1.p1, l1.p2), l2);
  }catch(std::invalid_argument){
    std::vector<Vec> leer (0);
    return leer;
  }
  double tv = teilverhaeltnis (l1.p1, l1.p2, is);
  if (tv<0 || tv>1) {
    std::vector<Vec> leer (0);
    return leer;
  }
  std::vector<Vec> nleer (1);
  nleer[0]=is;
  return nleer;
}

std::vector<Vec> Tribots::intersect (const Line& l1, const LineSegment& l2) throw (std::bad_alloc) {
  return intersect (l2,l1);
}

std::vector<Vec> Tribots::intersect (const LineSegment& l1, const LineSegment& l2) throw (std::bad_alloc) {
  Vec is;
  try{
    is = intersect (Line (l1.p1,l1.p2), Line(l2.p1,l2.p2));
  }catch(std::invalid_argument){
    std::vector<Vec> leer (0);
    return leer;
  }
  double tv1 = teilverhaeltnis (l1.p1, l1.p2, is);
  double tv2 = teilverhaeltnis (l2.p1, l2.p2, is);
  if (tv1<0 || tv1>1 || tv2<0 || tv2>1) {
    std::vector<Vec> leer (0);
    return leer;
  }
  std::vector<Vec> nleer (1);
  nleer[0]=is;
  return nleer;
}


LineSegment3D LineSegment3D::rotate (const Angle a) const throw () {
  LineSegment3D dest (*this);
  dest.s_rotate (a);
  return dest;
}

LineSegment3D LineSegment3D::translate (const Vec3D v) const throw () {
  LineSegment3D dest (*this);
  dest.s_translate (v);
  return dest;
}


*/

#if 0
TEST_GEOMETRY3D

using namespace Tribots;

int main(/*int args, char** argv*/){
  try{
  Vec3D v1(1.0,1.0, 0.0);
  Vec3D v2(2.0,2.1, 0.5);
  Line3D l1(v1, v2);
  
  Vec3D v3(1.0,1.0, 1.0);
  Vec3D v4(2.0,2.0, 1.0);
  Line3D l2(v3, v4);  
  Vec3D v5(-2,-2,-2);
  cout << "Test: "<<endl;
  cout << v1 << " und " <<v2<<" ergeben: "<<l1<<endl;
  cout << v3 << " und " <<v4<<" ergeben: "<<l2<<endl;
  cout << "intersect: " << intersect(l1, l2) << endl;  
  cout << "distance: " << l2.distance(v1) << endl;
  cout << "translate: " << l2.translate(v5) << endl;
  return 0;
  }
  catch(std::invalid_argument e){cout << e.what();}
}

#endif
