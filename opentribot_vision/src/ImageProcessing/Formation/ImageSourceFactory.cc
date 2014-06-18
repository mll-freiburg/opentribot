
#include "ImageSourceFactory.h"

using namespace Tribots;
using namespace std;


Tribots::ImageSourceFactory* Tribots::ImageSourceFactory::the_only_factory (NULL);

ImageSourceFactory::ImageSourceFactory () throw () {;}

ImageSourceFactory* ImageSourceFactory::get_image_source_factory () throw (std::bad_alloc) {
  if (!the_only_factory)
    the_only_factory = new ImageSourceFactory;
  return the_only_factory;
}

ImageSourceFactory::~ImageSourceFactory() throw () {;}

void ImageSourceFactory::sign_up (const std::string descriptor, ImageSourceBuilder* pointer) throw (std::bad_alloc) {
  list_of_plugins [descriptor] = pointer;
}

ImageSource* ImageSourceFactory::get_image_source (const std::string& descriptor, const ConfigReader& reader, const std::string& section) throw (TribotsException,bad_alloc,invalid_argument) {
  map<std::string, ImageSourceBuilder*>::iterator mit = list_of_plugins.find (descriptor);
  ImageSource* new_wm = NULL;
  if (mit!=list_of_plugins.end())
    new_wm = mit->second->get_image_source (descriptor, reader, section);
  else
    throw invalid_argument (string("unknown image source type ")+descriptor);
  return new_wm;
}

