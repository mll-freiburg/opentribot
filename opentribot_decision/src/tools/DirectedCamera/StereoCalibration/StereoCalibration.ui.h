/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you want to add, delete, or rename functions or slots, use
** Qt Designer to update this file, preserving your code.
**
** You should not define a constructor or destructor in this file.
** Instead, write your code in functions called init() and destroy().
** These will automatically be called by the form's constructor and
** destructor.
*****************************************************************************/

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <deque>
#include <qstatusbar.h>
#include "../../../Fundamental/ConfigReader.h"
#include "../../../Fundamental/Vec.h"
#include "../../../Fundamental/RingBuffer.h"
#include "../../../ImageProcessing/Formation/Painter.h"
#include "../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../ImageProcessing/PixelAnalysis/OmniCameraMapping.h"

using namespace std;
using namespace Tribots;

namespace {

  enum SCMode {
    pick,
    pickLine,
    pickStartLine,
    pickEndLine,
    pickMarkerGround,
    pickMarkerRaised,
    removeMarker
  };

  struct Markertype {
    Vec imagepos;
    Vec groundpos;
    bool selected;
    bool exists; // fuer den internen Abgleich
  };

  SCMode presentMode;
  Vec click1pos;

  Vec linePosStart [2];
  Vec linePosEnd [2];
  vector<Markertype> transitions [2];
  Vec latestMarkerGroundPos [2];

  Image* localImage [2];
  Image* fileImage [2];
  ImageWorldOmniMapping* omnimap;

  double edgeDetectionThreshold;

  unsigned int imageCount;

  RGBTuple yellowRGB = { 255, 255, 0 };
  RGBTuple redRGB = { 255, 0, 0 };
  RGBTuple greenRGB = { 0, 255, 0 };
  RGBTuple blueRGB = { 0, 0, 255 };
  RGBTuple blackRGB = { 0, 0, 0 };
  RGBTuple whiteRGB = { 255, 255, 255 };
  RGBTuple grayRGB = { 127, 127, 127 };
  RGBTuple magentaRGB = { 255, 0, 255 };
  RGBTuple cyanRGB = { 0, 255, 255 };

  void reorderTransitions (vector<Markertype>& transitions) {
    unsigned int firstUnselected=0;
    for (unsigned int j=0; j<transitions.size(); j++) {
      if (transitions[j].selected) {
        if (firstUnselected>=j) {
          firstUnselected=j+1;
        } else {
          Markertype mt = transitions[j];
          transitions[j]=transitions[firstUnselected];
          transitions[firstUnselected]=mt;
          firstUnselected++;
        }
      }
    }
  }

  vector<Vec> scanEdge (const Image& img, Vec start, Vec end, double threshold) {
    vector<Vec> result;
    Angle dir = (end-start).angle();
    double cosdir = cos (dir.get_rad());
    double sindir = sin (dir.get_rad());
    double step = 1.0/(abs(sindir)>abs(cosdir) ? abs(sindir) : abs(cosdir));
    RingBuffer<double> buffer (5);
    RingBuffer<char> maskOut (5);
    buffer[0]=buffer[1]=buffer[2]=buffer[3]=buffer[4]=127;
    maskOut[0]=maskOut[1]=maskOut[2]=maskOut[3]=maskOut[4]=true;
    int px=0;
    int py=0;
    double deriv1=0, deriv2=0;
    bool derivValid1=false, derivValid2=false;
    unsigned int count=0;
    unsigned int countmax=static_cast<unsigned int>((start-end).length()/step);
    while (count<=countmax) {
      px=static_cast<int>(floor(start.x+count*step*cosdir+0.5));
      py=static_cast<int>(floor(start.y+count*step*sindir+0.5));
      if (px<0 || py<0 || px>=img.getWidth() || py>=img.getHeight())
        break;
      YUVTuple yuv;
      img.getPixelYUV(px, py, &yuv);
      buffer.get()=yuv.y;
      maskOut.get()=false;
      double deriv = 0.5*(buffer[0]+buffer[1]-buffer[3]-buffer[4]);
      bool derivValid = (count>5) && (abs(deriv)>threshold) && !maskOut[0] && !maskOut[1] && !maskOut[2] && !maskOut[3] && !maskOut[4];
      if (count>=5 && abs(deriv1)>threshold && abs(deriv1)>abs(deriv2) && abs(deriv1)>abs(deriv) && derivValid1) {
        result.push_back (start+(count-3)*step*Vec(cosdir,sindir));
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

}

void StereoCalibration::init()
{
  omnimap=NULL;
  fileImage[0]=fileImage[1]=localImage[0]=localImage[1]=NULL;
  setImages(NULL, NULL);
  connect (imageWidget0, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(mousePressedInImage0(QMouseEvent*)));
  connect (imageWidget1, SIGNAL(mousePressed(QMouseEvent*)), this, SLOT(mousePressedInImage1(QMouseEvent*)));
  connect (imageWidget0, SIGNAL(mouseMoved(QMouseEvent*)), this, SLOT(mouseMovedInImage0(QMouseEvent*)));
  connect (imageWidget1, SIGNAL(mouseMoved(QMouseEvent*)), this, SLOT(mouseMovedInImage1(QMouseEvent*)));
  linePosStart[0]=linePosStart[1]=Vec(100,100);
  linePosEnd[0]=linePosEnd[1]=Vec(200,100);
  presentMode=pick;
  edgeDetectionThreshold=20;
  ConfigReader cfg;
  cfg.append_from_file (qApp->argv()[1]);
  string section = qApp->argv()[2];
  string distmarkerfile;
  vector<double> center;
  double height;
  cfg.get ((section+"::mapping_file").c_str(), distmarkerfile);
  cfg.get ((section+"::image_center").c_str(), center);
  cfg.get ((section+"::camera_height").c_str(), height);
  Vec3D origin(0,0,height);
  if (center.size()>=2) {
    omnimap = new ImageWorldOmniMapping (distmarkerfile.c_str(), origin, static_cast<int>(center[0]), static_cast<int>(center[1]));
  } else
    omnimap = new ImageWorldOmniMapping (distmarkerfile.c_str());
  imageCount=3;
  nextImage();
  markerout=NULL;
}

void StereoCalibration::destroy() {
  if (localImage[0]) delete localImage[0];
  if (localImage[1]) delete localImage[1];
  if (fileImage[0]) delete fileImage[0];
  if (fileImage[1]) delete fileImage[1];
  if (omnimap) delete omnimap;
}


void StereoCalibration::setImages( Tribots::Image * i0, Tribots::Image * i1)
{
  externalImage[0]=i0;
  externalImage[1]=i1;
  repaintImages();
}

void StereoCalibration::mousePressedInImage0( QMouseEvent * ev )
{
  mousePressedInImage (ev, 0);
}
void StereoCalibration::mousePressedInImage1( QMouseEvent * ev )
{
  mousePressedInImage (ev, 1);
}
void StereoCalibration::mousePressedInImage( QMouseEvent * ev, int id )
{
  Vec clickpos (ev->x(), ev->y());
  click1pos=clickpos;
  switch (presentMode) {
    case pick:
    case pickStartLine:
    case pickEndLine:
    case pickLine:
      {
        double d1=(clickpos-linePosStart[id]).length();
        double d2=(clickpos-linePosEnd[id]).length();
        if (d1<d2 && d1<5) {
          // Linienanfang
          linePosStart[id]=clickpos;
          presentMode=pickStartLine;
        } else if (d2<5) {
          // Linienende
          linePosEnd[id]=clickpos;
          presentMode=pickEndLine;
        } else {
          // Marker markieren
          bool doAdd = (ev->button()==Qt::LeftButton);
          bool doRemove = (ev->button()==Qt::RightButton);
          double mindist=1e20;
          unsigned int minindex=0;
          for (unsigned int j=0; j<transitions[id].size(); j++) {
            double d=(transitions[id][j].imagepos-clickpos).length();
            if (d<mindist && (transitions[id][j].selected==!doAdd)) {
              mindist=d;
              minindex=j;
            }
          }
          if (mindist<5) {
            transitions[id][minindex].selected=(doAdd ? true : (doRemove ? false : transitions[id][minindex].selected));
            if (doAdd)
              latestMarkerGroundPos[id]=transitions[id][minindex].groundpos=transitions[id][minindex].imagepos;
            reorderTransitions (transitions[id]);
            presentMode=pick;
          } else {
            // die ganze Linie packen
            LineSegment ls (linePosStart[id], linePosEnd[id]);
            if (ls.distance(clickpos)<5) {
              presentMode=pickLine;
            }
          }
        }
      }
      break;
    default:
      break;
  }
  repaintImages();
}


void StereoCalibration::mouseMovedInImage0( QMouseEvent * ev )
{
  mouseMovedInImage (ev,0);
}
void StereoCalibration::mouseMovedInImage1( QMouseEvent * ev )
{
  mouseMovedInImage (ev,1);
}
void StereoCalibration::mouseMovedInImage( QMouseEvent * ev, int id )
{
  if (ev->x()<0 || ev->y()<0 || ev->x()>=localImage[id]->getWidth() || ev->y()>=localImage[id]->getHeight())
    return;
  Vec clickpos (ev->x(), ev->y());
  bool doRepaint=false;
  switch (presentMode) {
    case pickEndLine:
      linePosEnd[id]=clickpos;
      doRepaint=true;
      break;
    case pickStartLine:
      linePosStart[id]=clickpos;
      doRepaint=true;
      break;
    case pickLine:
      linePosStart[id]+=(clickpos-click1pos);
      linePosEnd[id]+=(clickpos-click1pos);
      click1pos=clickpos;
      doRepaint=true;
      break;
    default:
      break;
  }
  if (doRepaint)
    repaintImages();
}


void StereoCalibration::repaintImages()
{
  for (int i=0; i<2; i++) {
    if (!externalImage[i])
      continue;
    if (localImage[i])
      delete localImage[i];
    localImage[i]=externalImage[i]->clone();

    vector<Vec> imgtransitions = scanEdge (*externalImage[i], linePosStart[i], linePosEnd[i], edgeDetectionThreshold);
    for (unsigned int j=0; j<transitions[i].size(); j++)
      transitions[i][j].exists=false;
    for (unsigned int k=0; k<imgtransitions.size(); k++) {
      bool found=false;
      for (unsigned int j=0; j<transitions[i].size(); j++) {
        if ((imgtransitions[k]-transitions[i][j].imagepos).length()<3) {
          transitions[i][j].exists=true;
          if (transitions[i][j].imagepos==transitions[i][j].groundpos)
            transitions[i][j].groundpos=imgtransitions[k];
          transitions[i][j].imagepos=imgtransitions[k];
          found=true;
        }
      }
      if (!found) {
        Markertype newmarker;
        newmarker.imagepos=imgtransitions[k];
        newmarker.groundpos=imgtransitions[k];
        newmarker.selected=false;
        newmarker.exists=true;
        transitions[i].push_back (newmarker);
      }
    }
    unsigned int j=0;
    while (j<transitions[i].size()) {
      if (!transitions[i][j].exists)
        transitions[i].erase (transitions[i].begin()+j);
      else
        j++;
    }

    Painter paint (*localImage[i]);
    paint.setColor (grayRGB);
    paint.drawLine (linePosStart[i], linePosEnd[i]);
    paint.setColor (cyanRGB);
    paint.drawCircle (linePosStart[i], 3);
    paint.drawCircle (linePosEnd[i], 3);

    for (unsigned int j=0; j<transitions[i].size(); j++) {
      if (transitions[i][j].selected) {
        if (transitions[i][j].imagepos==transitions[i][j].groundpos)
          paint.setColor (redRGB);
        else
          paint.setColor (magentaRGB);
      } else {
        paint.setColor (grayRGB);
      }
      paint.markRect (static_cast<int>(floor(transitions[i][j].imagepos.x+0.5)), static_cast<int>(floor(transitions[i][j].imagepos.y+0.5)),3);
      paint.setColor(cyanRGB);
      if (transitions[i][j].selected)
        paint.drawNumber (static_cast<int>(floor(transitions[i][j].imagepos.x+0.5))+5,static_cast<int>(floor(transitions[i][j].imagepos.y+0.5)), 12, j+1);
    }
  }
  if (localImage[0]) {
    imageWidget0->setImage (*localImage[0]);
    imageWidget0->update ();
  }
  if (localImage[1]) {
    imageWidget1->setImage (*localImage[1]);
    imageWidget1->update ();
  }
}


void StereoCalibration::setCalibrationLineSelected()
{
  presentMode=pick;
}
void StereoCalibration::markerGroundSelected()
{
  presentMode=pickMarkerGround;
}
void StereoCalibration::removeMarkerSelected()
{
  presentMode=removeMarker;
}
void StereoCalibration::markerRaisedSelected()
{
  presentMode=pickMarkerRaised;
}

void StereoCalibration::storeCorrespondences()
{
  if (!markerout)
    return;
  for (unsigned int j=0; j<transitions[0].size() && j<transitions[1].size(); j++) {
    if (transitions[0][j].selected && transitions[1][j].selected) {
      Vec worldpos = omnimap->map(transitions[0][j].groundpos);
//      Line3D line = omnimap->map3D(transitions[0][j].imagepos);
//      Vec zp = line.intersectZPlane (0).toVec();
//      double len = (zp-worldpos).length();
//      double slope = abs(line.p1.z-line.p2.z)/(line.p1-line.p2).toVec().length();
//      double height=slope*len;
      (*markerout) << worldpos << transitions[1][j].imagepos << endl;
    } else if (transitions[0][j].selected != transitions[1][j].selected) {
      statusBar()->message ("Error: markers has only been selected in one image. Skip it.", 5000);
    }
    transitions[0][j].selected=transitions[1][j].selected=false;
  }
}

void StereoCalibration::nextImage()
{
  if (fileImage[0]) delete fileImage[0];
  if (fileImage[1]) delete fileImage[1];
  fileImage[0]=fileImage[1]=NULL;
  if (static_cast<int>(imageCount+1)<qApp->argc()) {
    ifstream src1 (qApp->argv()[imageCount]);
    ifstream src2 (qApp->argv()[imageCount+1]);
    PPMIO io;
    ImageBuffer* sim = io.read (NULL, src1);
    if (!sim) {
      stringstream inout;
      inout << "Error in reading image file " << qApp->argv()[imageCount] << endl;
      string line;
      getline (inout, line);
      statusBar()->message (line.c_str());
      return;
    }
    fileImage[0] = new RGBImage (*sim);
    sim = io.read (NULL, src2);
    if (!sim) {
      stringstream inout;
      inout << "Error in reading image file " << qApp->argv()[imageCount+1] << endl;
      string line;
      getline (inout, line);
      statusBar()->message (line.c_str());
      return;
    }
    fileImage[1] = new RGBImage (*sim);
    imageCount+=2;
    setImages (fileImage[0], fileImage[1]);
  } else {
    statusBar()->message ("No next images available.", 5000);
  }
}


void StereoCalibration::writeCorrespondences( std::ostream & os )
{
  markerout = &os;
  os << "-1 -1 -1 -1" << endl;
}
