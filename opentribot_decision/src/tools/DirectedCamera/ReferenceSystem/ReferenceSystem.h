#ifndef _ReferenceSystem_h_
#define _ReferenceSystem_h_

#include <string>
#include <qapplication.h>
#include "../../../Fundamental/ConfigReader.h"

class SDL_mutex;
class ControlWidget;
class Visualization;
namespace Tribots {
class ImageProducer;
class Image;
class Vec;
class Image2WorldMapping3D;
class Vec3D;
class Line3D;
}

namespace TribotsTools {

  using namespace Tribots;
  using namespace std;
  
  class ImageWidget;

  class ReferenceSystem : QObject
  {
  Q_OBJECT
  
  public:
    ReferenceSystem(int argc, char* argv[]);
    virtual ~ReferenceSystem();
    
    void run();
    
  public slots:
    virtual void toggleVisualization(bool state);
    virtual void toggleCameraWindow(int id);
    void start();
    void stop();    
    
  protected:  
    ControlWidget* controlWidget;
    QApplication* app;
    
    Visualization* vis;
    
    ConfigReader* config;
    vector<string> producerConfigSections;
    
    bool running;
    int numCameras;
    ImageProducer** producers;
    ImageWidget** cameraWidgets;
    Image2WorldMapping3D** img2wrld;
    Vec3D* origins;
    
    SDL_mutex* inCycleMutex;
    
    void readConfiguration();
    void startCameras();
    void stopCameras();

    // folgende Methoden sind Kandidaten zur Auslagerung:    
    static void analyzeImage(Image* image, Vec* center, bool* found, 
                             bool draw= true);
    static int selectTwoClosestMeasurements(Vec3D* points, bool *found,
                                            Vec3D* origins, 
                                            int n, Line3D *line1, 
                                            Line3D *line2);
  };

}

#endif
