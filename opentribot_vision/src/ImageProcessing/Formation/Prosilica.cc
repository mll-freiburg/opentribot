
#include "Prosilica.h"
#include "ImageSourceFactory.h"
#include "../../Fundamental/stringconvert.h"
#include <sstream>

using namespace std;
using namespace Tribots;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::ImageSourceBuilder {
    public:
      Builder () {
        Tribots::ImageSourceFactory::get_image_source_factory ()->sign_up (string("Prosilica"), this);
      }
      Tribots::ImageSource* get_image_source (const std::string&, const Tribots::ConfigReader& reader, const std::string& section) throw (Tribots::TribotsException,std::bad_alloc) {
        return new Tribots::Prosilica (reader, section);
      }
  };
  Builder the_builder;
}

Prosilica::Prosilica (const ConfigReader& cfg, const std::string& section, bool force_blocking) throw (HardwareException, InvalidConfigurationException) {
    Y = new unsigned char [CAM_WIDTH*CAM_HEIGHT];
    U = new unsigned char [CAM_WIDTH*CAM_HEIGHT];
    V = new unsigned char [CAM_WIDTH*CAM_HEIGHT];

    int parlist[6]={128, 128, 128, 2048, 24, 1};

/*  initialize camera */
    if (init_camera(&handle,&camera,FRAMERATE)!=CAM_SUCCESS) {
        printf("test01 reports: could not init camera!!!\n");
        return;
    }

/*  set camera parameters */
    if (set_camera_param(&handle,&camera,parlist,1)!=CAM_SUCCESS) {
        printf("test01 reports: could not set camera parameters!!!\n");
        return;
    }
    
}

Prosilica::~Prosilica () throw () {
  stop_camera(handle,&camera);
  delete[] Y;
  delete[] U;
  delete[] V;
}

ImageBuffer Prosilica::getImage () throw (ImageFailed) {
  YUVImage* yuvImg = new YUVImage(CAM_WIDTH, CAM_HEIGHT);
  YUVTuple yuv;
  int idx;
  if (grab_image_YUV(Y,U,V,&camera)!=CAM_SUCCESS)
  {
    printf("test01 reports: could not grab image!!!\n");
  }
  for (int j=0;j<CAM_HEIGHT;j++)
  {
    int linestart = (j*CAM_WIDTH);
    for (int i=0;i<CAM_WIDTH;i++)
    {
      idx = linestart + i;
      yuv.y = Y[idx];
      yuv.u = U[idx];
      yuv.v = V[idx];
      yuvImg->setPixelYUV(i,j,yuv);
    }
  }
  
  return yuvImg->getImageBuffer();
}