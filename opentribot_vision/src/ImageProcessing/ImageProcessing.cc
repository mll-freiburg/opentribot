
#include "ImageProcessing.h"
#include "ImageProcessingFactory.h"
#include "../Structures/Journal.h"
#include "Formation/ImageProducer.h"
#include "cstring"

using namespace Tribots;
using namespace std;

const char* Tribots::ImageProcessing::get_image_processing_type () const throw () { return image_processing_descriptor; }

Tribots::ImageProcessing::~ImageProcessing () throw () {
  delete the_image_processing;
  delete [] image_processing_descriptor;
}

Tribots::ImageProcessing::ImageProcessing (const ConfigReader& vread, 
                                           const char* type, 
                                           const char* section, 
                                           const ImageProducer* producer)
  throw (TribotsException, bad_alloc) 
  : the_image_processing(NULL), configuration_list(vread) 
{
  string confline;
  try{
    really_change_image_processing_type (type, section, vread, producer);
  }catch(TribotsException& e){
    JERROR((std::string ("creating image processing of type ")+std::string(type)+std::string(" failed due to: ")+std::string(e.what())).c_str());
    throw (e);
  }

}

bool 
Tribots::ImageProcessing::change_image_processing_type (const char* ipt, const char* section, const ImageProducer* producer) throw () {
  return change_image_processing_type (ipt, section, configuration_list, producer);
}

bool Tribots::ImageProcessing::change_image_processing_type (const char* ipt, const char* section, const ConfigReader& vread, const ImageProducer* producer) throw () {
  try{
    really_change_image_processing_type (ipt, section, vread, producer);
  }catch(bad_alloc&){
    JWARNING("Change of image processing type failed due to lack of memory");
    return false;
  }catch(TribotsException&){
    JWARNING("Change of image processing type failed");
    return false;
  }
  return true;
}

void Tribots::ImageProcessing::really_change_image_processing_type (const char* ipt, const char* section, const ConfigReader& vread, const ImageProducer* producer) throw (TribotsException, bad_alloc) {
  ImageProcessingType* new_image_processing;
  char* new_descriptor;
  try{
    string ipts (ipt);
    string sections (section);
    new_image_processing=ImageProcessingFactory::get_image_processing_factory()->get_image_processing(ipts, sections, vread, producer);
  }catch(invalid_argument&){
    throw Tribots::InvalidConfigurationException ("requested image type"); // FIXME: wrong exception ?
  }
  
  new_descriptor = new char [strlen(ipt)+1];
  strcpy(new_descriptor,ipt);
  if (the_image_processing!=NULL) {
    delete the_image_processing;
    delete [] image_processing_descriptor;
  }
  the_image_processing=new_image_processing;
  image_processing_descriptor=new_descriptor;
}

