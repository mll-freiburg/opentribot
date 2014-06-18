#include "ReferenceSystem.h"
#include "ControlWidget.h"
#include "Visualization.h"
#include "../../../Fundamental/Vec3D.h"
#include "../../../Fundamental/geometry3Dgsl.h"
#include "../../../ImageProcessing/Formation/ImageProducer.h"
#include "../../../Structures/TribotsException.h"
#include "../../components/ImageWidget.h"
#include "../../../ImageProcessing/PixelAnalysis/Image2WorldMapping3D.h"

#include <SDL_thread.h>

#include <qaction.h>
#include <qstring.h>
#include <qpopupmenu.h>

namespace TribotsTools {

using namespace Tribots;
using namespace std;

ReferenceSystem::ReferenceSystem(int argc, char* argv[]) 
  : QObject(), vis(0), config(0), running(false)
{
  // Create application and main window
  app = new QApplication(argc, argv);
  controlWidget = new ControlWidget();
  controlWidget->show();
  app->setMainWidget(controlWidget);
  
  inCycleMutex = SDL_CreateMutex();

  // process Arguments
  if (argc != 4) {
    cerr << "Usage: (to fill in)" << endl;
    qApp->exit(1);
  }
  string configFileName = argv[1];
  
  // Open and read configuration file
  config = new ConfigReader(0);
  config->append_from_file(configFileName.c_str());
  readConfiguration();  

  // Read image-to-world lookup tables 
  img2wrld = new (Image2WorldMapping3D*)[numCameras];
  origins = new Vec3D[numCameras];
  for (int i=0; i < numCameras; i++) {
    img2wrld[i] = new Image2WorldMapping3D(argv[i+2]);
    origins[i] = img2wrld[i]->getOrigin();
  }

  // connect 3d Visualization on/off switch
  connect(dynamic_cast<QObject*>(controlWidget->window3D_VisualizationAction),
          SIGNAL(toggled(bool)),
          dynamic_cast<QObject*>(this), SLOT(toggleVisualization(bool)));
  controlWidget->window3D_VisualizationAction->setOn(false);
  
  // connect fileExitAction to end program (atm: via closing the apps
  // mainWindow)
  connect(dynamic_cast<QObject*>(controlWidget->fileExitAction), 
          SIGNAL(activated()),
          dynamic_cast<QObject*>(controlWidget),
          SLOT(hide()));
          
  // connect controlStart / -Stop actions
  connect(dynamic_cast<QObject*>(controlWidget->controlStartAction), 
          SIGNAL(activated()),
          dynamic_cast<QObject*>(this),
          SLOT(start()));
  connect(dynamic_cast<QObject*>(controlWidget->controlStopAction), 
          SIGNAL(activated()),
          dynamic_cast<QObject*>(this),
          SLOT(stop()));
}

void
ReferenceSystem::readConfiguration()
{
  if (config->get ("image_sources", producerConfigSections)<=0)
    throw InvalidConfigurationException("image_sources"); 
  numCameras = producerConfigSections.size();
}

ReferenceSystem::~ReferenceSystem()
{
  if (running) stop();
  if (config) delete config;
  if (vis) delete vis;
  for (int i=0; i < numCameras; i++) {
    delete img2wrld[i]; 
  }
  delete [] img2wrld;
  delete [] origins;
  delete controlWidget;
  delete app;
}

void
ReferenceSystem::start()
{
  SDL_mutexP(inCycleMutex);       // cycle should not start before finished 
  startCameras();
  running = true;
  SDL_mutexV(inCycleMutex);
}

void
ReferenceSystem::stop()
{
  SDL_mutexP(inCycleMutex);
  running = false;
  stopCameras();
  SDL_mutexV(inCycleMutex);
}

void
ReferenceSystem::run()
{
  Image* images[numCameras];
  do {
    qApp->processEvents();
    
    SDL_mutexP(inCycleMutex); //////////////////// processing cycle //////////
    if (running) {
      bool ok;
      do {
        ok = true;
        for (int i=0; i < numCameras; i++) {
          try {
            images[i] = producers[i]->nextImage();
          } catch (ImageFailed& e) {
	          cerr << e.what() << endl;
            ok = false;
            break;
          }
        }
      } while (! ok);       // solange wiederholen, bis alle bilder ok

      Vec3D centers[numCameras];
      bool found[numCameras];
      for (int i=0; i < numCameras; i++) {  // Bilder analysieren
        analyzeImage(images[i], &centers[i], &found[i]);
        if (found[i]) {
          centers[i] = 
            img2wrld[i]->map((int)centers[i].x, (int)centers[i].y);
          if (vis) {
            vis->set2DCenter(centers[i], i);
            vis->setOrigin(origins[i], i);
          }
        }
      }
      Line3D line1, line2;
      int linesFound = 
        selectTwoClosestMeasurements(centers, found, origins, numCameras, 
                                     &line1, &line2);
      if (linesFound==2) {
        cerr << "Lines: " << line1 << " and  " << line2 << endl;
        Vec3D center3D = intersect_least_squares(line1, line2);
        if (vis) {
          vis->set3DCenter(center3D);
        }
      }
      else {
        // object has not been detected in enough cameras
      }
      
      for (int i=0; i < numCameras; i++) {
        if (cameraWidgets[i]) {
          cameraWidgets[i]->setImage(*images[i]);
        }
      }
            
      for (int i=0; i < numCameras; i++) {
        delete images[i];
      }
    }
    SDL_mutexV(inCycleMutex); ////////////////////////////////////////////////
    if (!running) {
      usleep(10 * 1000); // processEvents only uses to much cpu
    }       
  } while (qApp->mainWidget()->isVisible());
}

void
ReferenceSystem::toggleVisualization(bool state)
{
  if (state) {
    if (!vis) {
      vis = new Visualization();
    }
  }
  else {
    if (vis) {
      SDL_mutexP(inCycleMutex);
      delete vis;
      vis = 0;
      SDL_mutexV(inCycleMutex);
    }
  }
}

void 
ReferenceSystem::toggleCameraWindow(int id)
{
  if (id < 0 || id >= numCameras) {
    return;
  }
  if (cameraWidgets[id]) {
    delete cameraWidgets[id];
    cameraWidgets[id] = 0;
    controlWidget->Window->setItemChecked(id, false);
  }
  else {
    QString caption = 
      QString("Camera ") + QString().setNum(id) + " - " + "Section: " +
      producerConfigSections[id];
    cameraWidgets[id] = new ImageWidget(0, "CameraWidget");
    cameraWidgets[id]->setCaption(caption.ascii());
    cameraWidgets[id]->show();
    controlWidget->Window->setItemChecked(id, true);
  }
}
  

void 
ReferenceSystem::startCameras()
{
  producers = new (ImageProducer*) [numCameras];
  for (int i=0; i < numCameras; i++) {
    producers[i] = new ImageProducer(*config, producerConfigSections[i]);
    controlWidget->Window->insertItem(QString("Camera ") + 
                                      QString().setNum(i), i, i);
  }
  controlWidget->Window->insertSeparator(numCameras);
  connect(dynamic_cast<QObject*>(controlWidget->Window), 
          SIGNAL(activated(int)),
          dynamic_cast<QObject*>(this), SLOT(toggleCameraWindow(int)));
  cameraWidgets = new (ImageWidget*)[numCameras];
  for (int i=0; i < numCameras; i++) {
    cameraWidgets[i] = 0;
  }
  
  controlWidget->controlStopAction->setEnabled(true);
  controlWidget->controlStartAction->setEnabled(false);
}

void
ReferenceSystem::stopCameras()
{
  controlWidget->Window->removeItemAt(numCameras);
  for (int i=0; i < numCameras; i++) {
    delete producers[i];
    controlWidget->Window->removeItem(i);
    if (cameraWidgets[i]) delete cameraWidgets[i];
  }
  disconnect(dynamic_cast<QObject*>(controlWidget->Window), 
             SIGNAL(activated(int)),
             dynamic_cast<QObject*>(this), SLOT(toggleCameraWindow(int)));
          
  delete [] cameraWidgets;          
  delete [] producers;
  controlWidget->controlStopAction->setEnabled(false);
  controlWidget->controlStartAction->setEnabled(true);
}

void
ReferenceSystem::analyzeImage(Image* image, Vec* center, bool* found,
                             bool draw)
{
  int dimX = image->getWidth();
  int dimY = image->getHeight();
  long centerX=0;
  long centerY=0;
  int n=0;
  RGBTuple green = { 0, 255, 0 };
  for (int x=0; x < dimX; x++) {
    for (int y=0; y < dimY; y++) {
      if (image->getPixelClass(x,y) == 1) {
        centerX += x; centerY += y; ++n;
        if (draw) {
          image->setPixelRGB(x,y, green);
        }
      }
    }
  }
  if (n == 0) {
    *found = false;
  }
  else {
    *found = true;
    *center = Vec(centerX / (double)n, centerY / (double)n);
  }
}


/** aeusserst haessliche, zu ueberarbeitende methode, die die zwei 
 *  kuerzesten Linien aus den gefundenen Punkten und Urspruengen 
 *  heraussucht.
 */
int 
ReferenceSystem::selectTwoClosestMeasurements(Vec3D* points, 
                                              bool *found,
                                              Vec3D* origins, 
                                              int n, Line3D *line1, 
                                              Line3D *line2)
{
  Line3D result[3];
  double distance[3];
  int count = 0;
  int i=0;
  for (; i < n && count < 2; i++) {
    if (found[i]) {
      result[count] = Line3D(origins[i], points[i]);
      distance[count++] = (points[i] - origins[i]).length();
    }
  }
  if (count < 2) {  
    return count;    // zu wenige objekte gesehen, keine 2 linien möglich
  }
  if (distance[0] > distance[1]) {
     distance[2] = distance[0];
     result[2] = result[0];
     distance[0] = distance[1];
     result[0] = result[1];
     distance[1] = distance[2];
     result[1] = result[2];
  }
  for (; i < n; i++) {
    if (found[i]) {
      double d = (points[i] - origins[i]).length();
      if (d < distance[0]) { 
        distance[1] = distance[0];
        result[1] = result[0];
        distance[0] = d;
        result[0] = Line3D(origins[i], points[i]);
      }
      else if (d < distance[1]) {
        distance[1] = d;
        result[1] = Line3D(origins[i], points[1]);
      }
    }
  }
  *line1 = result[0];
  *line2 = result[1]; 
  return 2;
}
}
