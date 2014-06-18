#include "Visualization.h"

#include <SDL/SDL.h>
#include <SDL/SDL_thread.h>

#include "OpenGLApp.h"

#include <iostream>

using namespace std;

Visualization::Visualization(int width, int height, int cameras)
{
  vis = new OpenGLApp(width, height, cameras);
  thread = SDL_CreateThread(startThread, this);
}

Visualization::~Visualization()
{
  vis->end();
  SDL_WaitThread(thread, 0);
  delete vis;
}

int 
Visualization::startThread(void* objectToStart)
{

  Visualization* tThis = 
    reinterpret_cast<Visualization*>(objectToStart);
  tThis->vis->run();

  return 0;
}

void 
Visualization::setOrigin(Vec3D origin, int camera)
{
  vis->setOrigin(origin, camera);
}

void 
Visualization::set2DCenter(Vec center, int camera)
{ 
  vis->set2DCenter(center, camera);
}

void 
Visualization::set3DCenter(Vec3D center)
{
  vis->set3DCenter(center);
}
