#include "OpenGLApp.h"

#include <iostream>

using namespace std;

OpenGLApp::OpenGLApp(int width, int height, int cameras)
  : numCameras(cameras), width(width), height(height), endSignaled(false), 
    hasFocus(false)
{
  camOrigins = new Vec3D(0., 0., 0.);
  objCenters2D = new Vec(0., 0.);
}

OpenGLApp::~OpenGLApp()
{
  delete [] objCenters2D;
  delete [] camOrigins;
}

void
OpenGLApp::end()
{
  endSignaled=true;
}

void 
OpenGLApp::setOrigin(Vec3D origin, int camera)
{
  camOrigins[camera] = origin;
}

void 
OpenGLApp::set2DCenter(Vec center, int camera)
{
  objCenters2D[camera] = center;
}

void 
OpenGLApp::set3DCenter(Vec3D center)
{
  objCenter3D = center;
}

void
OpenGLApp::run()
{
  init();
  while(! endSignaled) {
    doUserInput();
    doGraphics();
    usleep(10 * 1000);
  }
  cleanup();
}

// renderDevice has to be initialized INSIDE the thread that will run the 
// graphics loop. Therefore init is not called by the constructor or 
// by the user directly, but by the run-method to guarantee, initialization
// happens in the same thread!

void
OpenGLApp::init()  
{
  renderDevice = new RenderDevice();
  RenderDeviceSettings settings;
  settings.fsaaSamples = 1;
  settings.resizable = true;
  settings.width = width;
  settings.height = height;  
  renderDevice->init(settings, new Log());
  
  userInput = new UserInput();

  controller = new ManualCameraController(renderDevice, userInput);
  controller->setMoveRate(10);
  controller->setPosition(Vector3(10, 10, 10));
  controller->lookAt(Vector3(0,0,0));

  renderDevice->resetState();
  renderDevice->setColorClearValue(Color3(.1, .1, .1));

  controller->setActive(false);
}

void
OpenGLApp::cleanup()
{
  delete controller;
  delete userInput;
  renderDevice->cleanup();
  delete renderDevice;  
}

void
OpenGLApp::doUserInput()
{
  SDL_Event event;
  userInput->beginEvents();
  while(SDL_PollEvent(&event)) {
    switch(event.type) {
    case SDL_VIDEORESIZE: {
      renderDevice->notifyResize(event.resize.w, event.resize.h); // notify rd
      Rect2D full = Rect2D::xywh(0, 0,                            // and adapt
                                 renderDevice->getWidth(), 
                                 renderDevice->getHeight());  
      renderDevice->setViewport(full);                            // viewport
    } break;
    case SDL_KEYDOWN: {
      switch (event.key.keysym.sym) {
      case SDLK_ESCAPE:
        hasFocus = false;
        controller->setActive(false);
      break;
      case SDLK_TAB:
        hasFocus = ! controller->active();
        controller->setActive(! controller->active() );
      break;
      default: ;
      }
    } break;
    }    
    if (hasFocus) {
      userInput->processEvent(event);
    }
  }
  userInput->endEvents();
}

void
OpenGLApp::doGraphics()
{
  controller->doSimulation(0.03);
  camera.setCoordinateFrame(controller->getCoordinateFrame());

  renderDevice->beginFrame();

  renderDevice->clear();
  renderDevice->setProjectionAndCameraMatrix(camera);
  
  LightingParameters lp(toSeconds(11, 00, AM));
  GLight light = GLight::directional(lp.lightDirection,
                                     lp.lightColor);
  renderDevice->setLight(0, light);
  renderDevice->setAmbientLightColor(lp.ambient);
  renderDevice->enableLighting();

  Draw::axes(renderDevice, Color3::red(), Color3::yellow(), Color3::red(), 
	     5.0); 

  for (int i=0; i < numCameras; i++) {
    Vec3D vec3 = camOrigins[i];
    Sphere cam(Vector3(vec3.x, vec3.z, vec3.y), 1.);
    Draw::sphere(cam, renderDevice, Color4(.6, .6, .6));
    
    Vec vec2 = objCenters2D[i];
    Sphere center(Vector3(vec2.x, 0, vec2.y), .7);
    Draw::sphere(center, renderDevice, Color4(.8, .5, .8), Color4(.8, .5, .8));

    LineSegment line = 
      LineSegment::fromTwoPoints(Vector3(vec2.x, 0, vec2.y), 
				 Vector3(vec3.x, vec3.z, vec3.y));
    Draw::lineSegment(line, renderDevice, Color4(1., 1., 1.));
				    
  }  
  Box groundPlane( Vector3(0., 0., 0.), Vector3(10., 0., 10.) );
  Draw::box(groundPlane, renderDevice, Color4(1., 1., .0, .2));

  Sphere center(Vector3(objCenter3D.x, objCenter3D.z, objCenter3D.y), .5);
  Draw::sphere(center, renderDevice, 
	       Color4(.2, 1., .2, .6), Color4(.2, 1., .2, .0));

  renderDevice->endFrame();
}
