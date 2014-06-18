#ifndef _OpenGLApp_h_
#define _OpenGLApp_h_

#include <G3DAll.h>
#include "../../../Fundamental/Vec3D.h"
#include "../../../Fundamental/Vec.h"

using namespace Tribots;

class OpenGLApp {
public:
  OpenGLApp(int width, int height, int cameras);
  ~OpenGLApp();

  void run();

  void setOrigin(Vec3D origin, int camera);
  void set2DCenter(Vec center, int camera);
  void set3DCenter(Vec3D center);

  void end();
  
protected:
  int numCameras;
  int width, height;
  
  Vec3D* camOrigins;
  Vec*   objCenters2D;
  Vec3D  objCenter3D;

  bool endSignaled;

  // the following object pointers will be (initialized) inside the thread 
  // that executes run()
  RenderDevice*           renderDevice;
  CFontRef                font;
  UserInput*              userInput;
  ManualCameraController* controller;
  GCamera                 camera;
  
  bool hasFocus;

  void init();///< called by run(). Initializes renderDevice and related stuff.
  void cleanup();///< called by run(). Deinitializes renderDevice and related stuff.
  void doUserInput();
  void doGraphics();
};

class GLVis : public GApplet {
public:
  GLVis(OpenGLApp* app);
  ~GLVis();

  void doGraphics();

protected:
  OpenGLApp* app;
};

#endif
