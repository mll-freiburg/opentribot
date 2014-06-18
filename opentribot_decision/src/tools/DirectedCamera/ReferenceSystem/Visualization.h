#ifndef _Visualization_h_
#define _Visualization_h_

#include "../../../Fundamental/Vec3D.h"
#include "../../../Fundamental/Vec.h"

using namespace Tribots;

// Wrapper, that hides OpenGL related Stuff from user. Should be safe,
// although it seems that there are some naming conflicts between qt3 and
// g3d. Constructor starts a thread to run the continuous visualization routine
// of OpenGLApp and delegates all other methods to OpenGLApp.

class OpenGLApp;
class SDL_Thread;

class Visualization {
public:
  Visualization(int width=480, int height=320, int cameras=2);
  ~Visualization();

  void setOrigin(Vec3D origin, int camera);
  void set2DCenter(Vec center, int camera);
  void set3DCenter(Vec3D center);

protected:
  static int startThread(void* objectToStart);
  OpenGLApp  *vis;
  SDL_Thread *thread;
};

#endif
