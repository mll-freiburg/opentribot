
#include "CameraOptics.h"
#include <cmath>

using namespace Tribots;
using namespace std;


// ------------ Klasse DistortionMapping --------------
DistortionMapping::DistortionMapping (const CameraOptics& optics, unsigned int w, unsigned int h) throw (std::bad_alloc) :
     mapping(NULL) {
  width=w;
  height=h;
  rebuildMapping (optics);
}

DistortionMapping::~DistortionMapping () throw () {
  if (mapping)
    delete [] mapping;
}

unsigned int DistortionMapping::getWidth() const throw () { return width; }

unsigned int DistortionMapping::getHeight() const throw () { return height; }

Vec DistortionMapping::map(const Vec& p) const {
  int u=static_cast<int>(p.x+0.5);
  int v=static_cast<int>(p.y+0.5);
  return mapping[v*width+u];
}

void DistortionMapping::rebuildMapping (const CameraOptics& optics) throw (std::bad_alloc) {
  if (mapping)
    delete [] mapping;
  mapping = new Vec [width*height];
  for (unsigned int v=0; v<height; v++)
    for (unsigned int u=0; u<width; u++)
      mapping [v*width+u]=optics.distort(Vec(u,v));
}



// ------------ Klasse UndistortionMapping --------------
UndistortionMapping::UndistortionMapping (const CameraOptics& optics, unsigned int w, unsigned int h) throw (std::bad_alloc) :
     mapping(NULL) {
  width=w;
  height=h;
  rebuildMapping (optics);
}

UndistortionMapping::~UndistortionMapping () throw () {
  if (mapping)
    delete [] mapping;
}

unsigned int UndistortionMapping::getWidth() const throw () { return width; }

unsigned int UndistortionMapping::getHeight() const throw () { return height; }

Vec UndistortionMapping::map(const Vec& p) const {
  int u=static_cast<int>(p.x+0.5);
  int v=static_cast<int>(p.y+0.5);
  return mapping[v*width+u];
}

void UndistortionMapping::rebuildMapping (const CameraOptics& optics) throw (std::bad_alloc) {
  if (mapping)
    delete [] mapping;
  mapping = new Vec [width*height];

  // Baue durch Vorwaertsberechnung ein etwas groesseres Array auf:
  int offset=3;
  int offset2=200;
  int rowwidth=width+2*offset;
  int columnheight=height+2*offset;
  int totaloffset=offset+offset*rowwidth;
  Vec* imap = new Vec [rowwidth*columnheight];

  // Abbildung initialisieren
  for (int v=-offset; v<height+offset; v++) {
    for (int u=-offset; u<width+offset; u++) {
      imap[v*rowwidth+u+totaloffset]=Vec(-10000, -10000);
    }
  }

  // Abbildung durch Vorwaertsberechnung anfuellen
  for (int v=-offset2; v<height+offset2; v++) {
    for (int u=-offset2; u<width+offset2; u++) {
      Vec image=optics.distort(Vec(u,v));
      int ud = static_cast<int>(image.x+0.5);
      int vd = static_cast<int>(image.y+0.5);
      if (ud>=-offset && ud<width+offset && vd>=-offset && vd<height+offset)
        imap[vd*rowwidth+ud+totaloffset]=Vec(u,v);
    }
  }

  // Luecken in Abbildung schliessen durch gewichtete Mittelwertbildung
  double f = -2.0/(offset*offset);
  for (int v=0; v<height; v++) {
    for (int u=0; u<width; u++) {
      Vec result = imap[v*rowwidth+u+totaloffset];
      if (result.x<-5000) {
        result=Vec(0,0);
        double sumW = 0;
        for (int dv=-offset; dv<=offset; dv++) {
          for (int du=-offset; du<=offset; du++) {
            Vec p = imap[(v+dv)*rowwidth+(u+du)+totaloffset];
            if (p.x<-1e5)
              continue;
            double w=exp(f*(du*du+dv*dv));  // Gausssche Glaettung
            result+=w*p;
            sumW+=w;
          }
        }
        if (sumW!=0) {
          result/=sumW;
        }
      }
      mapping[v*width+u]=result;
    }
  }
  delete [] imap;
}


// -------------- Klasse CameraOptics ------------------

CameraOptics::CameraOptics () throw () {
  sx=sy=1;
  u0=v0=0;
  k1=k2=p1=p2=0;
  translation.x=translation.y=translation.z=0;
  rotationColumn1.x=rotationColumn2.y=rotationColumn3.z=1;
  rotationColumn1.y=rotationColumn2.z=rotationColumn3.x=0;
  rotationColumn1.z=rotationColumn2.x=rotationColumn3.y=0;
  recalculateHelpers ();
}

void CameraOptics::setIntrinsicParameters (double sx1, double sy1, double u01, double v01) throw () {
  sx=sx1;
  sy=sy1;
  u0=u01;
  v0=v01;
  recalculateHelpers ();
}

void CameraOptics::shiftPrinciplePoint (Vec delta) throw () {
  u0+=delta.x;
  v0+=delta.y;
  recalculateHelpers();
}

void CameraOptics::setExtrinsicParameters (Vec3D translation1, Vec3D rotation1, Vec3D rotation2) throw () {
  translation=translation1;
  rotationColumn1=rotation1;
  rotationColumn2=rotation2;
  rotationColumn3=rotation1.crossProduct (rotation2);
  recalculateHelpers ();
}

void CameraOptics::setDistortionParameters (double k11, double k21, double p11, double p21) throw () {
  k1=k11;
  k2=k21;
  p1=p11;
  p2=p21;
}

Vec CameraOptics::distort (Vec p) const throw () {
  double x = (p.x-u0)/sx;  // Einheitsebene
  double y = (p.y-v0)/sy;
  double r2 = x*x+y*y;  // Radius^2

  double xd = (1+k1*r2+k2*r2*r2)*x + 2*p1*x*y + p2*(r2+2*x*x);
  double yd = (1+k1*r2+k2*r2*r2)*y + 2*p2*x*y + p1*(r2+2*y*y);
  return Vec (u0+sx*xd, v0+sy*yd);
}

Vec CameraOptics::pinholeMap (Vec3D p) const throw (std::invalid_argument) {
  Vec3D q = p.x*rotationColumn1+p.y*rotationColumn2+p.z*rotationColumn3+translation;
  if (q.z<=0)
    throw invalid_argument ("Tribots::CameraOptics::pinholeMap: argument behind camera plane");
  return Vec (u0+sx*q.x/q.z, v0+sy*q.y/q.z);
}

Line3D CameraOptics::inversePinholeMap (Vec p) const throw () {
  return Line3D (-RtT, RtAi1*p.x+RtAi2*p.y+RtAi3mRtT);
}

void CameraOptics::inversePinholeMap (Angle& azimuth, Angle& inclination, Vec p) const throw () {
  Vec3D dir3d = RtAi1*p.x+RtAi2*p.y+RtAi3mRtT+RtT;
  azimuth = dir3d.toVec().angle();
  inclination = Angle::rad_angle (atan2(dir3d.z, dir3d.toVec().length()));
}

Vec CameraOptics::map (Vec3D p) const throw (std::invalid_argument) {
  Vec b = pinholeMap (p);
  return distort (b);
}

Vec3D CameraOptics::cameraOrigin () const throw () {
  return -RtT;
}

void CameraOptics::setRollPitchYaw (Angle roll, Angle pitch, Angle yaw) throw () {
  double cr=cos(roll.get_rad());
  double sr=sin(roll.get_rad());
  double cp=cos(pitch.get_rad());
  double sp=sin(pitch.get_rad());
  double cy=cos(yaw.get_rad());
  double sy=sin(yaw.get_rad());
  rotationColumn1 = Vec3D (cy*cp, sy*cp, -sp);
  rotationColumn2 = Vec3D (cy*sp*sr-sy*cr, sy*sp*sr+cy*cr, cp*sr);
  rotationColumn3 = rotationColumn1.crossProduct (rotationColumn2);
  recalculateHelpers ();
}

void CameraOptics::getRollPitchYaw (Angle& roll, Angle& pitch, Angle& yaw) const throw () {
  pitch = Angle::rad_angle (atan2 (-rotationColumn1.z, sqrt(rotationColumn1.x*rotationColumn1.x+rotationColumn1.y*rotationColumn1.y)));
  double cpitch = cos(pitch.get_rad());
  if (cpitch==0) {
    yaw=Angle::zero;
    if (pitch.in_between (Angle::zero, Angle::half)) {
      roll=Angle::rad_angle (atan2(rotationColumn2.x, rotationColumn1.x));
    } else {
      roll=Angle::rad_angle (-atan2(rotationColumn2.x, rotationColumn1.x));
    }
  } else {
    yaw = atan2 (rotationColumn1.y/cpitch, rotationColumn1.x/cpitch);
    roll = atan2 (rotationColumn2.z/cpitch, rotationColumn3.z/cpitch);
  }
}

Halfplane CameraOptics::nonObservableHalfplane () const throw (std::invalid_argument) {
  Vec n (-rotationColumn1.z, -rotationColumn2.z);
  double l2=n.squared_length();
  if (l2==0)
    throw std::invalid_argument ("Tribots::CameraOptics::nonObservableHalfplane: camera axis orthogonal to x-y-plane");
  Vec cp = -translation.z/l2*n;
  return Halfplane (cp, n);
}

void CameraOptics::writeParametersToStream (std::ostream& os) const throw () {
  // Reihenfolge:
  // - Intrinsische Parameter (sx, sy, u0, v0)
  // - Verzerrungsparameter (k1, k2, p1, p2)
  // - Translationsvektor (tx, ty, tz)
  // - Roll/Yaw/Pitch-Winkel in Grad (r,y,p)
  os << sx << ' ' << sy << ' ' << u0 << ' ' << v0 << '\n';
  os << k1 << ' ' << k2 << ' ' << p1 << ' ' << p2 << '\n';
  os << translation.x << ' ' << translation.y << ' ' << translation.z << '\n';
  Angle roll, yaw, pitch;
  getRollPitchYaw (roll, pitch, yaw);
  os << roll.get_deg() << ' ' << yaw.get_deg() << ' ' << pitch.get_deg() << '\n';
}

bool CameraOptics::readParametersFromStream (std::istream& is) throw () {
  double roll, yaw, pitch;
  is >> sx >> sy >> u0 >> v0
      >> k1 >> k2 >> p1 >> p2
      >> translation.x >> translation.y >> translation.z
      >> roll >> yaw >> pitch;
  if (is.eof())
    return false;
  setRollPitchYaw (Angle::deg_angle(roll), Angle::deg_angle(pitch), Angle::deg_angle(yaw));
  recalculateHelpers ();
  return true;
}

void CameraOptics::recalculateHelpers () throw () {
  RtT.x=rotationColumn1*translation;
  RtT.y=rotationColumn2*translation;
  RtT.z=rotationColumn3*translation;
  RtAi1.x=rotationColumn1.x/sx;
  RtAi1.y=rotationColumn2.x/sx;
  RtAi1.z=rotationColumn3.x/sx;
  RtAi2.x=rotationColumn1.y/sy;
  RtAi2.y=rotationColumn2.y/sy;
  RtAi2.z=rotationColumn3.y/sy;
  RtAi3mRtT.x=-rotationColumn1.x*u0/sx-rotationColumn1.y*v0/sy+rotationColumn1.z-RtT.x;
  RtAi3mRtT.y=-rotationColumn2.x*u0/sx-rotationColumn2.y*v0/sy+rotationColumn2.z-RtT.y;
  RtAi3mRtT.z=-rotationColumn3.x*u0/sx-rotationColumn3.y*v0/sy+rotationColumn3.z-RtT.z;
}
