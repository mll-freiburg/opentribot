
#include "ImageProcessingFactory.h"

using namespace Tribots;
using namespace std;

Tribots::ImageProcessingFactory* Tribots::ImageProcessingFactory::the_only_factory (NULL);

ImageProcessingFactory::ImageProcessingFactory () throw () {;}

ImageProcessingFactory* ImageProcessingFactory::get_image_processing_factory () throw (std::bad_alloc) {
  if (!the_only_factory)
    the_only_factory = new ImageProcessingFactory;
  return the_only_factory;
}

ImageProcessingFactory::~ImageProcessingFactory() throw () {;}

void ImageProcessingFactory::sign_up (const std::string descriptor, ImageProcessingBuilder* pointer) throw (std::bad_alloc) {
  list_of_plugins [descriptor] = pointer;
}

ImageProcessingType* ImageProcessingFactory::get_image_processing (const std::string descriptor, const std::string section, const ConfigReader& reader, const ImageProducer* producer) throw (TribotsException,bad_alloc,invalid_argument) {
  map<std::string, ImageProcessingBuilder*>::iterator mit = list_of_plugins.find (descriptor);
  ImageProcessingType* new_wm = NULL;
  if (mit!=list_of_plugins.end())
    new_wm = mit->second->get_image_processing (descriptor, section, reader, NULL, producer);
  else
    throw invalid_argument (string("unknown image processing type ")+descriptor);

  return new_wm;
}

  
