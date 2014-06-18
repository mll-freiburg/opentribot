
#include "DistanceCalibrationImageAnalysis.h"
#include "../../ImageProcessing/Formation/Painter.h"
#include "../../ImageProcessing/Formation/ColorTuples.h"
#include "../../Fundamental/Vec.h"
#include "../../Fundamental/RingBuffer.h"
#include <cmath>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

namespace {
  double mod2pi (double a) {
    if (a>0)
      return a-2*M_PI*(floor(a/(2*M_PI)));
    return a+2*M_PI*(ceil(-a/(2*M_PI)));
  }
  double square (double x) {
    return x*x;
  }

  RGBTuple rgbColors [] = {
    { 64, 64, 64 },
    { 255, 64, 64 },
    { 0, 0, 255 },
    { 196, 196, 196 },
    { 255, 255, 0 },
    { 255, 255, 0 },
    { 255, 255, 0 },
    { 255, 255, 0 },
    { 255, 255, 0 },
    { 255, 255, 0 }
  };
}


AngleKalmanFilter::AngleKalmanFilter (double va1, double va2, double va3, double bt) : varalpha(va1), vartheta(va2), varobs(va3), beta(bt) {
  setInitial (0,0);
}
AngleKalmanFilter::~AngleKalmanFilter () {;}

void AngleKalmanFilter::setInitial (double a, double t) {
  alphahat=alpha=a;
  thetahat=theta=t;
  p11=p11hat=varalpha;
  p12=p12hat=0;
  p22=p22hat=vartheta;
}
void AngleKalmanFilter::predict (double thetasoll, double deltat) {
  alphahat = alpha+theta*deltat;
  thetahat = beta*theta+(1-beta)*thetasoll;
  p11hat = p11+2*p12*deltat+p22*deltat*deltat+varalpha;
  p12hat = beta*p12+beta*p22*deltat;
  p22hat = beta*beta*p22+vartheta;
}
void AngleKalmanFilter::update (double d) {
  double diff = mod2pi (d-alphahat);
  if (diff>M_PI)
    diff-=2*M_PI;
  double k1 = p11hat/(p11hat+varobs);
  double k2 = p12hat/(p11hat+varobs);
  alpha = alphahat+k1*diff;
  theta = thetahat+k2*diff;
  p11 = (1-k1)*p11hat;
  p12 = (1-k1)*p12hat;
  p22 = -k2*p12hat+p22hat;
}

double AngleKalmanFilter::getAngle () const {
  return mod2pi (alpha);
}
double AngleKalmanFilter::getVelocity () const {
  return theta;
}
double AngleKalmanFilter::getPredictedAngle () const {
  return mod2pi (alphahat);
}
double AngleKalmanFilter::getPredictedVelocity () const {
  return thetahat;
}



DistanceCalibrationImageAnalysis::DistanceCalibrationImageAnalysis ()
  : robotMask (NULL), kfilter (0.00173611,0.000625,0.00694444,0.7)
{
  setRotation (0);
  setCenter (320,240);
  setSearchDirection (0);
  pmangle= 15.0f *M_PI/180; //winkeloeffnung in der nach der roten markierung gesucht wird (-+)
}

DistanceCalibrationImageAnalysis::~DistanceCalibrationImageAnalysis () {;}

std::vector<MarkerLog> DistanceCalibrationImageAnalysis::nextImage (Tribots::Image& img, bool showCenter, bool showDirection, bool showMask, bool showClasses, bool showTransitions) {
  vector<MarkerLog> result = findTransitions (img, thetasoll);
  if (showMask && robotMask)
    writeMask (img);
  if (showClasses)
    writeColorClasses (img);
  if (showTransitions)
    writeTransitions (img, result);
  if (showDirection)
    writeDirection (img);
  if (showCenter)
    writeCenter (img);
  return result;
}

void DistanceCalibrationImageAnalysis::setRotation (double velocity) {
  thetasoll=-velocity;
  timestampUpdate=true;
}

void DistanceCalibrationImageAnalysis::setCenter (int cx, int cy) {
  centerX=cx;
  centerY=cy;
}

void DistanceCalibrationImageAnalysis::setMask (Tribots::RobotMask* pt) {
  robotMask=pt;
}

void DistanceCalibrationImageAnalysis::setSearchDirection (Tribots::Angle d) {
  kfilter.setInitial (d.get_rad(), thetasoll);
}

void DistanceCalibrationImageAnalysis::getCenter (int& x, int& y) const {
  x=centerX;
  y=centerY;
}

void DistanceCalibrationImageAnalysis::setEdgeDetectionSensitivity (unsigned int v) {
  edgeDetectionSensitivity=v;
}


std::vector<MarkerLog> DistanceCalibrationImageAnalysis::findTransitions (const Image& img, double thetasoll) {
  vector<MarkerLog> markers;
  double deltat = 1e-6*static_cast<double>(img.getTimestamp().diff_usec (latestUpdateTime));
  kfilter.predict (thetasoll, deltat);
  Angle searchdir = Angle::rad_angle (kfilter.getPredictedAngle ());
  cout << "Searching in Direction"<< searchdir.get_deg()<<endl; 

  double freeratio = checkDirection (img, searchdir);
  if (freeratio<0.15) {
    return markers;  // weitgehend ausmaskierte Richtung, ueberspringen um nicht den Kalman-Filter in Probleme zu bringen
  }
  if (timestampUpdate) {
    latestUpdateTime=img.getTimestamp();
    timestampUpdate=false;
  }

  // Suchstrahlen in Richtung searchdir+/-10 Grad, Farbuebergaenge bestimmen, orange Flaeche extrahieren mit 3-4 blauen Flaechen davor
  vector<double> dirsOkay;
  vector<vector<MarkerLog> > rayMarkers;
  for (double a=searchdir.get_rad()-pmangle; a<searchdir.get_rad()+pmangle; a+=0.01) {
//  { double a=searchdir.get_rad();  // nur zu Testzwecken statt Zeile obendran
    vector<MarkerLog> localMarkers;
    Angle dir = Angle::rad_angle (a);
    vector<ScanArea> colorScanResult = scan (img, dir, 3.0);  // nach Farben scannen
    vector<double> edgeScanResult = scanEdge (img, dir, edgeDetectionSensitivity);  // nach Kanten scannen

    // rote Patches suchen und Anfang und Ende des ersten Patches (mit kleinen Luecken) bestimmen
    double redPatchBegin=0;
    double redPatchEnd=0;
    for (unsigned int i=0; i<colorScanResult.size(); i++) {
      if (colorScanResult[i].color==ColorRedPatch) {
        if (redPatchBegin<1) {
          redPatchBegin = colorScanResult[i].begin;
          redPatchEnd = colorScanResult[i].end;
        } else if (colorScanResult[i].begin<redPatchEnd+10) {
          redPatchEnd = colorScanResult[i].end;
        } else {
          break;
        }
      }
    }
    bool directionOkay=(redPatchBegin*480.0/img.getHeight()>100) && (redPatchEnd*480.0/img.getHeight()<300);

    cout << "DirectionOkay "<< directionOkay<<endl;

    // Pruefen, ob die Kanten alternierend hell-dunkel/dunkel-hell sind
    bool edgeAlternating=true;
    int lastEdgeSign=0;
    for (unsigned int i=0; i<edgeScanResult.size(); i++) {


      int edgeSign = (edgeScanResult[i]>=0 ? +1 : -1);
     
      //cout << "edgeScanResult"<<edgeScanResult[i]<<endl;
      edgeAlternating &= (edgeSign*lastEdgeSign<=0);
      lastEdgeSign=edgeSign;
    }
    directionOkay &= edgeAlternating;

    // wenn Plausubilitaetscheck okay, dann Marker einfuegen
    if (directionOkay) {
      int edgeBegin=-1;
      int edgeEnd=-1;
      for (unsigned int i=0; i<edgeScanResult.size(); i++) {
        if (edgeScanResult[i]<=-redPatchBegin+10 && edgeScanResult[i]>-redPatchBegin-1)
          edgeBegin=i;
        if (edgeScanResult[i]>redPatchEnd-1 && edgeScanResult[i]<=redPatchEnd+10)
          edgeEnd=i;
      }
      if (edgeBegin==-1 || (edgeEnd!=-1 && edgeEnd-edgeBegin!=1))
        directionOkay=false;

      MarkerLog nm;
      nm.angle = Angle::rad_angle (a);
      for (int i=0; i<edgeBegin; i++) {
        nm.type = (edgeScanResult[i]>0 ? MarkerLog::BW : MarkerLog::WB);
        nm.distance = abs(edgeScanResult[i]);
        localMarkers.push_back (nm);
      }
      if (edgeBegin>=0) {
        nm.type = MarkerLog::WR;
        nm.distance = abs(edgeScanResult[edgeBegin]);
        localMarkers.push_back (nm);
        redPatchBegin=nm.distance;
      }
      if (edgeEnd>=0) {
        nm.type = MarkerLog::RW;
        nm.distance = abs(edgeScanResult[edgeEnd]);
        localMarkers.push_back (nm);
        redPatchEnd=nm.distance;
      } else {
        nm.type = MarkerLog::RW;
        nm.distance = redPatchEnd;
        localMarkers.push_back (nm);
      }
      nm.type = MarkerLog::RM;
      nm.distance = 0.5*(redPatchBegin+redPatchEnd);
      localMarkers.push_back (nm);

      if (directionOkay) {
        dirsOkay.push_back (a);
        rayMarkers.push_back (localMarkers);
      }
    }
  }

  if (dirsOkay.size()>0) {
    double median_angle = (dirsOkay.size()%2==0 ? (dirsOkay[dirsOkay.size()/2]+dirsOkay[dirsOkay.size()/2-1])/2 : dirsOkay[dirsOkay.size()/2]);
    for (unsigned int i=0; i<dirsOkay.size(); i++) {
      if (dirsOkay[i]+0.035>median_angle && median_angle+0.035>dirsOkay[i]) {
        markers.insert (markers.end(), rayMarkers[i].begin(), rayMarkers[i].end());
      }
    }
    kfilter.update (median_angle);
    latestUpdateTime = img.getTimestamp();
  }
  return markers;
}

double DistanceCalibrationImageAnalysis::checkDirection (const Tribots::Image& img, Tribots::Angle dir) {
  double cosdir = cos (dir.get_rad());
  double sindir = sin (dir.get_rad());
  double step = 1.0/(abs(sindir)>abs(cosdir) ? abs(sindir) : abs(cosdir));
  unsigned int numValid=0;
  double distance = 0;
  while (true) {
    int posX = static_cast<int>(floor(centerX+distance*cosdir+0.5));
    int posY = static_cast<int>(floor(centerY+distance*sindir+0.5));
    if (posX<0 || posY<0 || posX>=img.getWidth() || posY>=img.getHeight())
      break;
    if (robotMask && robotMask->isValid (posX, posY))
      numValid++;
    distance+=step;
  }
  return step*numValid/img.getHeight();
}

std::vector<ScanArea> DistanceCalibrationImageAnalysis::scan (const Tribots::Image& img, Tribots::Angle dir, double minlen) {
  std::vector<ScanArea> result;
  double cosdir = cos (dir.get_rad());
  double sindir = sin (dir.get_rad());
  double step = 1.0/(abs(sindir)>abs(cosdir) ? abs(sindir) : abs(cosdir));
  ScanArea area;
  area.begin=0;
  area.end=0;
  area.color=ColorIgnore;
  bool valid=false;
  double distance = 0;
  while (true) {
    int posX = static_cast<int>(floor(centerX+distance*cosdir+0.5));
    int posY = static_cast<int>(floor(centerY+distance*sindir+0.5));
    if (posX<0 || posY<0 || posX>=img.getWidth() || posY>=img.getHeight()) {
      if (valid)
        result.push_back (area);
      break;
    }
    bool pixelValid = (robotMask ? robotMask->isValid (posX, posY) : true);
    unsigned char colorClass = (pixelValid ? img.getPixelClass (posX, posY) : ColorIgnore);
    if (valid && pixelValid && colorClass==area.color) {
      area.end=distance;
    } else if (valid) {
      if (area.end-area.begin>=minlen)
        result.push_back (area);
      area.begin=area.end=distance;
      valid=pixelValid;
      area.color=colorClass;
    } else if (pixelValid) {
      area.begin=area.end=distance;
      valid=true;
      area.color=colorClass;
    } else {
      area.end=distance;
    }
    distance+=step;
  }
  return result;
}

vector<double> DistanceCalibrationImageAnalysis::scanEdge (const Image& img, Angle dir, double threshold) {
  vector<double> result;
  double cosdir = cos (dir.get_rad());
  double sindir = sin (dir.get_rad());
  double step = 1.0/(abs(sindir)>abs(cosdir) ? abs(sindir) : abs(cosdir));
  RingBuffer<double> buffer (5);
  RingBuffer<char> maskOut (5);
  buffer[0]=buffer[1]=buffer[2]=buffer[3]=buffer[4]=127;
  maskOut[0]=maskOut[1]=maskOut[2]=maskOut[3]=maskOut[4]=true;
  int px=centerX;
  int py=centerY;
  double deriv1=0, deriv2=0;
  bool derivValid1=false, derivValid2=false;
  unsigned int count=0;
  while (true) {
    px=static_cast<int>(floor(centerX+count*step*cosdir+0.5));
    py=static_cast<int>(floor(centerY+count*step*sindir+0.5));
    if (px<0 || py<0 || px>=img.getWidth() || py>=img.getHeight())
      break;
    YUVTuple yuv;
    img.getPixelYUV(px, py, &yuv);
    buffer.get()=yuv.y;
    bool valid = (robotMask ? robotMask->isValid (px, py) : true);
    maskOut.get()=!valid;
    double deriv = 0.5*(buffer[0]+buffer[1]-buffer[3]-buffer[4]);
    bool derivValid = (count>5) && (abs(deriv)>threshold) && !maskOut[0] && !maskOut[1] && !maskOut[2] && !maskOut[3] && !maskOut[4];
    if (count>=5 && abs(deriv1)>threshold && abs(deriv1)>abs(deriv2) && abs(deriv1)>abs(deriv) && derivValid1) {
      if (deriv1>0) {
        result.push_back (static_cast<double>(count-3)*step);
      } else {
        result.push_back (-static_cast<double>(count-3)*step);
      }
    }

    count++;
    buffer.step(-1);
    maskOut.step(-1);
    deriv2=deriv1;
    deriv1=deriv;
    derivValid2=derivValid1;
    derivValid1=derivValid;
  }
  return result;
}


void DistanceCalibrationImageAnalysis::writeCenter (Image& img) {
  Painter pt (img);
  pt.setPen (Painter::PEN_SOLID);
  RGBTuple white;
  white.r=white.g=white.b=255;
  pt.setColor (white);
  pt.markCrossbar (centerX, centerY, 5);
}

void DistanceCalibrationImageAnalysis::writeDirection (Image& img) {
  Painter pt (img);
  RGBTuple yellow ={ 255, 255, 0 };
  RGBTuple orange = { 255, 248, 0 };
  pt.setColor (yellow);
  pt.setPen (Painter::PEN_STEPPED);
  pt.drawLine (Tribots::Vec (centerX, centerY), Tribots::Vec (centerX+200*cos(kfilter.getPredictedAngle()), centerY+200*sin(kfilter.getPredictedAngle())));
  pt.setPen (Painter::PEN_SOLID);
  pt.drawLine (Tribots::Vec (centerX, centerY), Tribots::Vec (centerX+200*cos(kfilter.getAngle()), centerY+200*sin(kfilter.getAngle())));
  pt.setColor (orange);
  pt.setPen (Painter::PEN_STEPPED);
  pt.drawLine (Tribots::Vec (centerX, centerY), Tribots::Vec (centerX+200*cos(kfilter.getPredictedAngle()-pmangle), centerY+200*sin(kfilter.getPredictedAngle()-pmangle)));
  pt.drawLine (Tribots::Vec (centerX, centerY), Tribots::Vec (centerX+200*cos(kfilter.getPredictedAngle()+pmangle), centerY+200*sin(kfilter.getPredictedAngle()+pmangle)));
}

void DistanceCalibrationImageAnalysis::writeMask (Image& img) {
  if (!robotMask)
    return;
  unsigned int iw=img.getWidth();
  unsigned int ih=img.getHeight();
  RGBTuple rgb;
  for (unsigned int i=0; i<iw; i++) {
    for (unsigned int j=0; j<ih; j++) {
      if (!robotMask->isValid(i,j)) {
        img.getPixelRGB(i,j,&rgb);
        rgb.r = (static_cast<int>(rgb.r)+175)/3;
        rgb.g = (static_cast<int>(rgb.g)+175)/3;
        rgb.b = (static_cast<int>(rgb.b)+175)/2;
        img.setPixelRGB(i,j,rgb);
      }
    }
  }
}

void DistanceCalibrationImageAnalysis::writeTransitions (Image& img, const std::vector<MarkerLog>& markers) {
  Painter pt (img);
  pt.setPen (Painter::PEN_SOLID);
  RGBTuple white, blue, red, magenta, yellow;
  white.r=white.g=white.b=blue.b=red.r=magenta.b=magenta.r=255;
  yellow.r=yellow.g=196;
  blue.r=blue.g=red.g=red.b=magenta.g=yellow.b=0;

  for (unsigned int i=0; i<markers.size(); i++) {
    RGBTuple markerColor;
    bool doPaint=true;
    switch (markers[i].type) {
      case MarkerLog::WB : markerColor = blue; break;
      case MarkerLog::BW : markerColor = yellow; break;
      case MarkerLog::WR : markerColor = red; break;
      case MarkerLog::RW : markerColor = magenta; break;
      default : markerColor = white; doPaint=false; break;
    }
    if (doPaint) {
      pt.setColor (markerColor);
      Vec pos (centerX+markers[i].distance*cos(markers[i].angle.get_rad()), centerY+markers[i].distance*sin(markers[i].angle.get_rad()));
      pt.drawLine (pos+5*Vec::unit_vector (markers[i].angle+Angle::quarter), pos-5*Vec::unit_vector (markers[i].angle+Angle::quarter));
      pt.drawLine (pos+5*Vec::unit_vector (markers[i].angle), pos-5*Vec::unit_vector (markers[i].angle));
    }
  }
}

void DistanceCalibrationImageAnalysis::writeColorClasses (Image& img) {
  for (int i=0; i<img.getWidth(); i++) {
    for (int j=0; j<img.getHeight(); j++) {
      unsigned int cls = img.getPixelClass (i,j);
      if (cls!=ColorIgnore) {
        img.setPixelRGB (i, j, rgbColors[cls]);
      }
    }
  }
}
