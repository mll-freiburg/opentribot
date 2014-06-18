
#include "DirectionalCalibration.h"
#include <cmath>

using namespace Tribots;
using namespace std;

DirectionalCalibration::Correspondence::Correspondence () throw () {;}

DirectionalCalibration::Correspondence::Correspondence (const Correspondence& c) throw () {
   operator= (c);
}

DirectionalCalibration::Correspondence::Correspondence (Vec w, Vec i) throw () :
    world(w),
    image(i) {;}

const DirectionalCalibration::Correspondence& DirectionalCalibration::Correspondence::operator= (const DirectionalCalibration::Correspondence& c) throw () {
  world=c.world;
  image=c.image;
  return *this;
}


DirectionalCalibration::DirectionalCalibration() throw () {;}

DirectionalCalibration::~DirectionalCalibration() throw () {;}

const CameraOptics& DirectionalCalibration::getCameraOptics () const throw () {
  return cameraOptics;
}

Image* DirectionalCalibration::undistortImage (Image* src) throw (std::bad_alloc) {
  int width = src->getWidth();
  int height = src->getHeight();
  int offset = 100;
  Image* dest = new YUVImage (width+2*offset, height+2*offset);
  for (int u=-offset; u<width+offset; u++) {
    for (int v=-offset; v<height+offset; v++) {
      Vec pd = cameraOptics.distort (Vec(u,v));
      int ud = static_cast<int>(pd.x+0.5);
      int vd = static_cast<int>(pd.y+0.5);
      YUVTuple yuv = {0,0,0};
      if (ud>=0 && vd>= 0 && ud<width && vd<height)
        src->getPixelYUV(ud, vd, &yuv);
      dest->setPixelYUV(offset+u, offset+v, yuv);
    }
  }
  RGBTuple rgb = {255,0,0};
  for (int u=0; u<width; u++) {
    dest->setPixelRGB(offset+u, offset, rgb);
    dest->setPixelRGB(offset+u, offset+height-1, rgb);
  }
  for (int v=0; v<height; v++) {
    dest->setPixelRGB(offset, offset+v, rgb);
    dest->setPixelRGB(offset+width-1, offset+v, rgb);
  }
  return dest;
}

Image* DirectionalCalibration::flattenImage (Image* src) throw (std::bad_alloc) {
  int width = src->getWidth();
  int height = src->getHeight();

  int dWidth = 400;
  int dHeight = 400;
  Image* dest = new YUVImage (2*dWidth+1, 2*dHeight+1);
  bool allObservable=false;
  Halfplane nonobservable (Vec(0,0), Vec(1,0));
  try{
    nonobservable = cameraOptics.nonObservableHalfplane ();
  }catch(std::invalid_argument&){
    allObservable=true;
  }
  for (int u=-dWidth; u<=dWidth; u++) {
    for (int v=-dHeight; v<=dHeight; v++) {
      YUVTuple yuv = {0,0,0};
      Vec3D wp (20*u,20*v,0);
      if (allObservable || !nonobservable.is_inside (wp.toVec())) {
        try{
          Vec pd = cameraOptics.map (wp);
          int ud = static_cast<int>(pd.x+0.5);
          int vd = static_cast<int>(pd.y+0.5);
          if (ud>=0 && vd>=0 && ud<width && vd<height)
            src->getPixelYUV(ud, vd, &yuv);
        }catch(invalid_argument&) {
          yuv.y=255;  // Fall sollte eigentlich bereits durch nonobservable abgefangen sein, zur Sicherheit trotzdem
        }
      } else {
        yuv.y=255;
      }
      dest->setPixelYUV(u+dWidth, dHeight-v, yuv);
    }
  }
  RGBTuple rgb = {255,0,0};
  for (int u=0; u<=2*dWidth; u++) {
    dest->setPixelRGB(u, dHeight, rgb);
  }
  for (int v=0; v<=2*dHeight; v++) {
    dest->setPixelRGB(dWidth, v, rgb);
  }
  return dest;
}

void DirectionalCalibration::setImageDimensions (unsigned int w, unsigned int h) throw () {
  width=w;
  height=h;
}

void DirectionalCalibration::readMarkers (const char* filename) throw (std::bad_alloc) {
  int pointsPerImage=0, images=0;
  ifstream in(filename);
  if (!in)
    throw std::invalid_argument (string("invalid argument in Tribots::DirectionalCalibration::calibrate: error while opening file ")+filename);

  in >> width >> height >> images >> pointsPerImage;

  unsigned int totalPoints=0;
  for(int image_count=0; image_count<images || images<0; image_count++) {
    if (in.eof())
      break;
    deque<Correspondence> newimage;
    for(int point_count=0; point_count<pointsPerImage || pointsPerImage<0; point_count++) {
      Correspondence cn;
      in >> cn.world.x >> cn.world.y >> cn.image.x >> cn.image.y;
      if (cn.image.x<0 && cn.image.y<0)
        break;   // vorzeitiger Abbruch der Punkte fuer ein Bild
      if (in.eof())
        break;
      newimage.push_back(cn);
      totalPoints++;
    }
    addMarkers (newimage);
  }
}

void DirectionalCalibration::addMarkers (const std::deque<Correspondence>& c) throw (std::bad_alloc) {
  if (c.size()==0)
    return;
  markers.push_back (c);
}

void DirectionalCalibration::calibrate () throw (std::invalid_argument, std::bad_alloc) {
  // Anzahl Ebenen, Anzahl Punkte, Punkte in opencv-Datenformat umwandeln:
  int numImages=markers.size();
  int numMarkers [numImages];
  int numMarkersTotal=0;
  for (unsigned int i=0; i<markers.size(); i++) {
    numMarkers[i]=markers[i].size();
    numMarkersTotal+=markers[i].size();
  }

//  CvMemStorage* mem = cvCreateMemStorage ();
  CvPoint3D32f worldPoints [numMarkersTotal];
  CvPoint2D32f imagePoints [numMarkersTotal];
  int k=0;
  for (int i=0; i<numImages; i++) {
    for (int j=0; j<numMarkers[i]; j++) {
      worldPoints [k] = cvPoint3D32f (markers[i][j].world.x, markers[i][j].world.y, 0);
      imagePoints [k] = cvPoint2D32f (markers[i][j].image.x, markers[i][j].image.y);
      ++k;
    }
  }

  // Kalibrierparameter in opencv-Datenformat erzeugen:
  float* fTransVects = new float [3*numImages];  // numImages x (3x1)-Vektor
  float* fRotMatrs = new float [9*numImages];  // numImages x (3x1)-Vektor
  float* fCameraMatrix = new float [9];
  float* fDistortion = new float [4];

  CvVect32f transVects = fTransVects;
  CvMatr32f rotMatrs = fRotMatrs;
  CvMatr32f cameraMatrix = fCameraMatrix;
  CvVect32f distortion = fDistortion;
  CvSize imageSize = cvSize (width,height);

  // Kalibrierung:
  cvCalibrateCamera (
                     numImages,
                     numMarkers,
                     imageSize,
                     imagePoints,
                     worldPoints,
                     distortion,
                     cameraMatrix,
                     transVects,
                     rotMatrs,
                     0);

  // Ergebnisse auswerten:
  Vec3D transVec;
  Vec3D rotVec1;
  Vec3D rotVec2;
  Vec3D rotVec3;
  transVec.x=transVects[3*(numImages-1)];
  transVec.y=transVects[3*(numImages-1)+1];
  transVec.z=transVects[3*(numImages-1)+2];
  rotVec1.x=rotMatrs[9*(numImages-1)];
  rotVec2.x=rotMatrs[9*(numImages-1)+1];
  rotVec3.x=rotMatrs[9*(numImages-1)+2];
  rotVec1.y=rotMatrs[9*(numImages-1)+3];
  rotVec2.y=rotMatrs[9*(numImages-1)+4];
  rotVec3.y=rotMatrs[9*(numImages-1)+5];
  rotVec1.z=rotMatrs[9*(numImages-1)+6];
  rotVec2.z=rotMatrs[9*(numImages-1)+7];
  rotVec3.z=rotMatrs[9*(numImages-1)+8];
  cameraOptics.setExtrinsicParameters (transVec, rotVec1, rotVec2);
  cameraOptics.setIntrinsicParameters (fCameraMatrix[0], fCameraMatrix[4], fCameraMatrix[2], fCameraMatrix[5]);
  cameraOptics.setDistortionParameters (fDistortion[0], fDistortion[1], fDistortion[2], fDistortion[3]);

  cerr << "ORIGIN (in mm):\n";
  cerr << '[' << cameraOptics.cameraOrigin() << "]\n";
  cerr << "ANGLES (Roll,Pitch,Yaw in Grad):\n";
  Angle roll, pitch, yaw;
  cameraOptics.getRollPitchYaw(roll, pitch, yaw);
  cerr << '[' << roll.get_deg() << ' ' << pitch.get_deg() << ' ' << yaw.get_deg() << ']' << endl;
  cerr << "EXAMPLE MAPPINGS:\n";
  for (double x=-1000; x<1001; x+=1000)
    for (double y=2000; y<4001; y+=1000) {
      Vec3D world (x,y,0);
      try{
        cerr << world << " => " << cameraOptics.map (world) << endl;
      }catch(std::invalid_argument&e){
        cerr << world << " behind camera" << endl;
      }
    }
//  cerr << "CHECK LAST IMAGE:\n";
//  for (unsigned int i=0; i<markers.back().size(); i++) {
//    Vec image = markers.back()[i].image;
//    Vec world = markers.back()[i].world;
//    Vec estimate = cameraOptics.map (world);
//    cout << image << " | " << world << " | " << estimate << " | " << (estimate-image).length() << '\n';
//  }


  delete [] fTransVects;
  delete [] fRotMatrs;
  delete [] fCameraMatrix;
  delete [] fDistortion;
//  cvReleaseMemStorage (&mem);
}
