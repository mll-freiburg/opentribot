
#include "CalibrationRecorder.h"
#include "../../../ImageProcessing/Calibration/Convolution.h"
#include "../../../ImageProcessing/Calibration/edgeDetection.h"
#include "../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../ImageProcessing/Formation/IIDC.h"
#include "../../../ImageProcessing/Formation/Painter.h"
#include "../../../Fundamental/Time.h"
#include "../../../Structures/TribotsException.h"
#include <sstream>
#include <cstdlib>
#include <qapplication.h>

using namespace std;
using namespace Tribots;
using namespace TribotsTools;

namespace {

  template <class ITERATOR>
  bool isAround (Vec p, double r, ITERATOR& begin, ITERATOR& end) {
    ITERATOR end2=end;
    end=begin;
    bool firstFound=false;
    for (ITERATOR it = begin; it!=end2; it++) {
      if ((it->x-p.x)*(it->x-p.x)+(it->y-p.y)*(it->y-p.y)<r*r) {
        if (!firstFound) {
          firstFound=true;
          begin=it;
        }
        end=it;
      }
    }
    if (begin==end)
      return false;
    end++;
    return true;
  }

  bool linelencomp (const deque<ImageCoordinate>& a1, const deque<ImageCoordinate>& a2) { return a1.size()>a2.size(); }

  double minDist (const std::deque<ImageCoordinate>& c1, const std::deque<ImageCoordinate>& c2, double sign) {
    if (sign>0) {
      double d2=c1.front().distance(c2.back());
      double d3=c1.back().distance(c2.front());
      return d2<d3 ? d2 : d3;
    } else {
      double d1=c1.front().distance(c2.front());
      double d4=c1.back().distance(c2.back());
      return d1<d4 ? d1 : d4;
    }
  }

}


CalibrationRecorder::CalibrationRecorder (const ConfigReader& cfg, const char* hostname, unsigned int port) {
  widget = new TribotsTools::ImageWidget;
  qApp->setMainWidget(widget);
  widget->show();
  panel = new CalibrationRecorderControlPanel;
  panel->show();

  regionBefore = new Pixelset;
  regionWork = new Pixelset;
  regionAfter = new Pixelset;

  // Verbindung zu MarkerDisplay aufbauen:
  serverPort=port;
  image = oldImage=NULL;
  string camerasection="";
  cfg.get ("camerasection", camerasection);
  camera = new IIDC (cfg, camerasection.c_str(), true);
  mode=neutral;
  hostInfo = gethostbyname(hostname);
  if (hostInfo == NULL)
    throw TribotsException ("problem interpreting host");
  socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);
  if (socketDescriptor < 0)
    throw TribotsException ("cannot create socket");
  serverAddress.sin_family = hostInfo->h_addrtype;
  memcpy((char *) &serverAddress.sin_addr.s_addr,
         hostInfo->h_addr_list[0], hostInfo->h_length);
  serverAddress.sin_port = htons(serverPort);
  if (connect(socketDescriptor,
              (struct sockaddr *) &serverAddress,
              sizeof(serverAddress)) < 0)
    throw TribotsException ("cannot connect to server");
  if (send (socketDescriptor, "blackScreen", 11,0)<0)
    throw TribotsException ("cannot send");
  char buffer [100];
  if (recv(socketDescriptor, buffer, 100,0)<0)
    throw TribotsException ("cannot receive");
  markerout=NULL;
}

CalibrationRecorder::~CalibrationRecorder () throw () {
  if (image)
    delete image;
  if (oldImage)
    delete oldImage;
  if (camera)
    delete camera;
  delete regionBefore;
  delete regionAfter;
  delete regionWork;
}

void CalibrationRecorder::communicate () {
  std::string sendmessage="";
  stringstream inout;
  switch (mode) {
    case neutral:
      return;
    case redScreen:
      sendmessage="redScreen";
      break;
    case greenScreen:
      sendmessage="greenScreen";
      break;
    case blueScreen:
      sendmessage="blueScreen";
      break;
    case redMarker:
      inout << "redMarker " << px << ' ' << py << endl;
      getline (inout, sendmessage);
      break;
    case blackMarker:
      inout << "calibrationMarker " << px << ' ' << py << endl;
      getline (inout, sendmessage);
      break;
  }
  if (send (socketDescriptor, sendmessage.c_str(), sendmessage.length(), 0)<0)
    throw TribotsException ("Send failed.");
  char buffer [100];
  int len=0;
  len=recv (socketDescriptor, buffer, 100,0);
  if (len<0)
    throw TribotsException ("Receive failed.");
  string receivemessage (buffer, len);
  if (receivemessage.substr(0,6)=="Marker") {
    unsigned int pos = receivemessage.find(' ', 7);
    trueMarkerPosition.x=atof(receivemessage.substr(7,pos-7).c_str());
    trueMarkerPosition.y=atof(receivemessage.substr(pos+1,receivemessage.length()-pos-1).c_str());
  }
}

void CalibrationRecorder::captureImage () {
  if (oldImage)
    delete oldImage;
  oldImage=image;
  camera->getImage ();  // ein Bild auslassen, koennte veraltet sein
  camera->getImage ();  // ein Bild auslassen, koennte veraltet sein
  camera->getImage ();  // ein Bild auslassen, koennte veraltet sein
  camera->getImage ();  // ein Bild auslassen, koennte veraltet sein
  ImageBuffer imbuf = camera->getImage ();
  image=new RGBImage (imbuf);
  imageWidth=image->getWidth();
  imageHeight=image->getHeight();
}

void CalibrationRecorder::processImage () {
  lastMarkerValid=false;
  double sensitivity = (static_cast<double>(panel->sensitivity)+100.0)/100.0;
  RGBTuple red = { 255,0,0 };
  RGBTuple green = { 0,255,0 };
  RGBTuple yellow = { 255,255,0 };
  RGBTuple blue = { 0,0,255 };
  Painter paint (*image);
  switch (mode) {
    case redScreen:
      setPixelsetAll (*regionBefore, image->getWidth(), image->getHeight());
      regionAfter->clear();
      findReddishPixels (*regionAfter, *image, sensitivity);
      drawPixelset (*image, *regionAfter, yellow);
      break;
    case greenScreen:
      regionWork->clear();
      regionAfter->clear();
      findGreenishPixels (*regionWork, *image, sensitivity);
      intersectSortedSets (*regionAfter, *regionBefore, *regionWork);
      drawPixelset (*image, *regionAfter, yellow);
      break;
    case blueScreen:
      regionWork->clear();
      regionAfter->clear();
      findBluishPixels (*regionWork, *image, sensitivity);
      intersectSortedSets (*regionAfter, *regionBefore, *regionWork);
      drawPixelset (*image, *regionAfter, yellow);
      break;
    case redMarker:
      regionWork->clear();
      regionAfter->clear();
      findReddishPixels (*regionWork, *image, sensitivity);
      intersectSortedSets (*regionAfter, *regionBefore, *regionWork);
      drawPixelset (*image, *regionAfter, yellow);
      gravityCenter = centerOfGravity (*regionAfter);
      if (gravityCenter.x==0 && gravityCenter.y==0)
        gravityCenter=Vec(-1000,-1000);  // fuer den Fall dass kein roter Bereich gefunden wurde
      paint.setColor (blue);
      paint.markCrosshair (static_cast<int>(gravityCenter.x+0.5), static_cast<int>(gravityCenter.y+0.5), 10);
      break;
    case blackMarker:
    {
      GrayLevelImage ggsrc (*image);
      std::deque<std::deque<ImageCoordinate> > lines;
      std::deque<std::deque<ImageCoordinate> > linesC;
      canny (lines, ggsrc, 5, 6, 20);
      for (unsigned int i=0; i<lines.size(); i++) {
        deque<unsigned int> breakpoints;
        breakLine (breakpoints, lines[i]);
        deque<deque<ImageCoordinate> > lines2;
        splitLineSorted (lines2, lines[i], breakpoints);
        for (unsigned int j=0; j<lines2.size(); j++) {
          if (lines2[j].size()>5) {
            deque<ImageCoordinate>::iterator it1=lines2[j].begin();
            deque<ImageCoordinate>::iterator it2=lines2[j].end();
            if (isAround (gravityCenter, 50, it1, it2)) {
              deque<ImageCoordinate> nl;
              linesC.push_back (nl);
              linesC.back().assign (it1, it2);
            }
          }
        }
      }
      if (linesC.size()>=4) {
        sort (linesC.begin(), linesC.end(), linelencomp);

        // beste Kombination der 4 Linien suchen
        unsigned int linesNum = (linesC.size()<8 ? linesC.size() : 8);
        Vec orientation [8];
        for (unsigned int i=0; i<linesNum; i++)
         orientation[i]=(linesC[i].front().toVec()-linesC[i].back().toVec()).normalize();

        unsigned int b1=99, b2=99, b3=99, b4=99;
        unsigned bestSolution = 1000;
        Vec intersection;
        for (unsigned int i1=0; i1<linesNum; i1++) {
          for (unsigned int i2=i1+1; i2<linesNum; i2++) {
            double orientationMatch1 = abs(orientation[i1]*orientation[i2]);
            if (orientationMatch1<0.5)
              continue; // Kanten sind mehr als 60Grad auseinander
            double minimalDistance1 = minDist (linesC[i1], linesC[i2], orientation[i1]*orientation[i2]);
            if (minimalDistance1>20)
              continue;  // Kanten sind zu weit voneinander entfernt
            std::deque<ImageCoordinate> pxl1 = linesC[i1];
            pxl1.insert (pxl1.end(), linesC[i2].begin(), linesC[i2].end());
            LineSegment ls1 = estimateLineSegment (pxl1.begin(), pxl1.end());
            Line l1 (ls1.getStart(), ls1.getEnd());
            Vec dir1 = (ls1.getStart()-ls1.getEnd()).normalize();
            for (unsigned int i3=0; i3<linesNum; i3++) if (i3!=i1 && i3!=i2) {
              for (unsigned int i4=i3+1; i4<linesNum; i4++) if (i4!=i1 && i4!=i2) {
                try{
                  double orientationMatch2 = abs(orientation[i3]*orientation[i4]);
                  if (orientationMatch1+orientationMatch2<1.5)
                    continue; // Kanten sind zu weit auseinander
                  double minimalDistance2 = minDist (linesC[i3], linesC[i4], orientation[i1]*orientation[i2]);
                  if (minimalDistance1+minimalDistance2>30)
                    continue;  // Kanten sind zu weit voneinander entfernt
                  std::deque<ImageCoordinate> pxl2 = linesC[i3];
                  pxl2.insert (pxl2.end(), linesC[i4].begin(), linesC[i4].end());
                  LineSegment ls2 = estimateLineSegment (pxl2.begin(), pxl2.end());
                  Line l2 (ls2.getStart(), ls2.getEnd());
                  Vec norm2 = (ls2.getStart()-ls2.getEnd()).normalize().rotate_quarter();
                  double mm = dir1*norm2;
                  if (mm<0.5)
                    continue;  // Kanten bilden kein Kreuz
                  Vec pi = intersect (l1, l2);
                  double md=1e300;
                  for (unsigned int i=0; i<regionAfter->size(); i++) {
                    double d=((*regionAfter)[i].toVec()-pi).length();
                    if (d<md)
                      md=d;
                  }
                  if (md>5)
                    continue;  // Punkt ausserhalb des roten Kreises
                  if (i1+i2+i3+i4<bestSolution) {
                    bestSolution=i1+i2+i3+i4;
                    intersection=pi;
                    b1=i1;
                    b2=i2;
                    b3=i3;
                    b4=i4;
                  }
                }catch(invalid_argument&){;}
              }
            }
          }
        }

        for (unsigned int i=0; i<linesC.size(); i++) {
          LineSegment ls = estimateLineSegment (linesC[i].begin(), linesC[i].end());
          bool sel = (i==b1 || i==b2 || i==b3 || i==b4);
          drawLine (*image, ls.getStart(), ls.getEnd(), (sel ? red : green), yellow);
        }

        if (bestSolution<1000) {
          cerr << "Schnittpunkt: " << intersection << endl;
          paint.setColor (blue);
          paint.markRect (static_cast<int>(intersection.x+0.5), static_cast<int>(intersection.y+0.5), 5);
          CalibrationMarker mk;
          mk.truePosition=trueMarkerPosition;
          mk.imagePosition=intersection;
          mk.imageNumber=imageNumber;
          markers.push_back (mk);
        } else {
          cerr << "Problem: die laengsten Linienstuecke bilden kein Kreuz\n";
        }
      } else {
        cerr << "Problem: nur " << linesC.size() << " Linienstuecke gefunden\n";
      }
      break;
    }
    default: // nothing, just display image
      break;
  }
}

void CalibrationRecorder::displayImage () {
  if (image)
    widget->setImage (*image);
  qApp->processEvents();
}

bool CalibrationRecorder::nextStep () {
  if (panel->doBack) {
    panel->doBack=false;
    panel->doNext=false;
    panel->doContinue=false;
    mode=neutral;
    unsigned int delMarkers=0;
    while (markers.size()>0 && markers.back().imageNumber==imageNumber) {
      markers.pop_back();
      delMarkers++;
    }
    cerr << "Remove " << delMarkers << '\n';
  }
  switch (mode) {
    case neutral:
      if (markers.size()>0 && markerout) {
        (*markerout) << imageWidth << ' ' << imageHeight << " -1 -1\n";
        for (unsigned int i=0; i<markers.size(); i++) {
          (*markerout) << markers[i].truePosition << markers[i].imagePosition << '\n';
        }
        (*markerout) << flush;
        markers.clear();
      }
      panel->setMessage ("put camera and monitor into right position...");
      if (panel->doContinue || panel->doNext) {
        panel->doNext=false;
        mode=redScreen;
      }
      break;
    case redScreen:
      panel->setMessage ("segment display area...");
      if (panel->doSkip) {
        mode=greenScreen;
        panel->doSkip=false;
      } else if (panel->doContinue || panel->doNext) {
        panel->doNext=false;
        mode=greenScreen;
        Pixelset* swp=regionBefore;
        regionBefore=regionAfter;
        regionAfter=swp;
      }
      break;
    case greenScreen:
      if (panel->doSkip) {
        mode=blueScreen;
        panel->doSkip=false;
      } else if (panel->doContinue || panel->doNext) {
        panel->doNext=false;
        mode=blueScreen;
        Pixelset* swp=regionBefore;
        regionBefore=regionAfter;
        regionAfter=swp;
      }
      break;
    case blueScreen:
      if (panel->doSkip) {
        mode=redMarker;
        regionOfInterest=(*regionBefore);
        panel->doSkip=false;
      } else if (panel->doContinue || panel->doNext) {
        panel->doNext=false;
        mode=redMarker;
        Pixelset* swp=regionBefore;
        regionBefore=regionAfter;
        regionAfter=swp;
        regionOfInterest=(*regionBefore);
      }
      px=py=4;
      break;
    case redMarker:
      panel->setMessage ("record markers...");
      if (panel->doSkip || panel->doContinue || panel->doNext) {
        panel->doNext=false;
        mode=blackMarker;
      }
      break;
    case blackMarker:
      panel->setMessage ("continue with next image...");
      bool saveMarker=lastMarkerValid && (!panel->doSkip) && (panel->doContinue || panel->doNext);
      bool nextImage=panel->doContinue || panel->doNext || panel->doSkip;
      if (saveMarker) {
        markers.push_back (lastMarker);
        cerr << "store marker\n";
      }
      if (nextImage) {
        panel->doNext=false;
        panel->doSkip=false;
        mode=redMarker;
        (*regionBefore)=regionOfInterest;
        px+=2;
        if (px>16) {
          px=4;
          py+=2;
        }
        if (py>16) {
          imageNumber++;
          mode=neutral;
          panel->doContinue=false;
        }
      }
  }
  return panel->doQuit;
}

void CalibrationRecorder::loop () {
  bool doQuit=false;
  while (!doQuit) {
    communicate ();
    captureImage ();
    processImage ();
    displayImage ();
    doQuit=nextStep ();
  }
}

void CalibrationRecorder::writeMarkers (std::ostream& os) {
  markerout = &os;
}
