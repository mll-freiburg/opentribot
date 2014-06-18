
#include "GrabbingThread.h"
#include <unistd.h>
#include "ColorTools.h"


using namespace TribotsTools;
using namespace Tribots;
using namespace std;

Tribots::POSIXMutex GrabbingThread::mutexImageSourceAccess;

namespace {

  POSIXMutex mutex_goon;  // kritischer Abschnitt fuer goon-Anfrage
  POSIXConditional cond_goon;  // warten bei !goon
  POSIXMutex mutex_frame_buffer;  // kritischer Abschnitt bei Zugriff auf prepared_buffer

}


TribotsTools::GrabbingThread::GrabbingThread(ImageSource* imgSrc)
  : imageSource(imgSrc), image(0), classifier(NULL) {
  bufferedImage[0]=bufferedImage[1]=bufferedImage[2]=NULL;
  image = new RGBImage (imageSource->getImage());
  bufferedImage[0] = new RGBImage(image->getWidth(), image->getHeight());
  bufferedImage[1] = new RGBImage(image->getWidth(), image->getHeight());
  bufferedImage[2] = new RGBImage(image->getWidth(), image->getHeight());
  client_buffer = prepared_buffer = 0;
}

TribotsTools::GrabbingThread::~GrabbingThread() {
  if (image) delete image;
  if (bufferedImage[0]) delete bufferedImage [0];
  if (bufferedImage[1]) delete bufferedImage [1];
  if (bufferedImage[2]) delete bufferedImage [2];
}

void GrabbingThread::start () {
  contGrabbing();
  POSIXThread::start();
}

void TribotsTools::GrabbingThread::main ()
{
  while(1) {
    checkCancel ();
    mutex_goon.lock();
    bool goon2 = goon;
    mutex_goon.unlock();

    if (!goon2) {
      cond_goon.wait ();
      continue;
    }

    if (image)
      delete image;
    mutexImageSourceAccess.lock();
    try{
      image = new RGBImage (imageSource->getImage());
    }catch(Tribots::ImageFailed&) {
      mutexImageSourceAccess.unlock();
      image=NULL;
      cerr << "Image failed!\n";
      continue;
    }
    mutexImageSourceAccess.unlock();

    mutex_frame_buffer.lock();
    unsigned free_buffer_index = (client_buffer!=0 && prepared_buffer!=0 ? 0 : (client_buffer!=1 && prepared_buffer!=1 ? 1 : 2));
    Tribots::Image* freebuffer = bufferedImage[free_buffer_index];
    mutex_frame_buffer.unlock();

    // Vorab-Version ohne Klassifizierung:
    if (!classifier) {
      RGBTuple rgb;
      rgb.r = 0;
      rgb.g = 0;
      rgb.b = 0;
      for (int x=0; x < image->getWidth(); x++) {
        for (int y=0; y < image->getHeight(); y++) {
          image->getPixelRGB(x,y, &rgb);
          freebuffer->setPixelRGB(x,y,rgb);
        }
      }
    } else {
      image->setClassifier(classifier);
      unsigned char c;
      RGBTuple rgb;
      rgb.r = 0;
      rgb.g = 0;
      rgb.b = 0;
      for (int x=0; x < image->getWidth(); x++) {
        for (int y=0; y < image->getHeight(); y++) {
          c = image->getPixelClass(x,y);
          if (c == COLOR_IGNORE) {
            image->getPixelRGB(x,y, &rgb);
            freebuffer->setPixelRGB(x,y,rgb);
          } else {
            freebuffer->setPixelRGB(x,y, colorInfos.classList[c]->color);
          }
        }
      }
    }

    mutex_frame_buffer.lock();
    prepared_buffer = free_buffer_index;
    mutex_frame_buffer.unlock();
  }
}

Tribots::Image& TribotsTools::GrabbingThread::getImage() {
  mutex_frame_buffer.lock ();
  client_buffer = prepared_buffer;
  mutex_frame_buffer.unlock ();
  return *(bufferedImage[client_buffer]); 
};

void TribotsTools::GrabbingThread::stopGrabbing () {
  mutex_goon.lock();
  goon=false;
  mutex_goon.unlock();
}

void TribotsTools::GrabbingThread::contGrabbing () {
  mutex_goon.lock();
  goon=true;
  mutex_goon.unlock();
  cond_goon.signal();
}

void GrabbingThread::cancel () {
  cond_goon.signal();
  POSIXThread::cancel();
}

void GrabbingThread::setClassifier (Tribots::ColorClassifier* classifier1) {
  classifier=classifier1;
}
