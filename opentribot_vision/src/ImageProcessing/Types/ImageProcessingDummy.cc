
#include "ImageProcessingDummy.h"
#include "../ImageProcessingFactory.h"
#include "../../WorldModel/WorldModel.h"

using namespace Tribots;
using namespace std;


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public ImageProcessingBuilder {
    static Builder the_builder;
  public:
    Builder () {
      ImageProcessingFactory::get_image_processing_factory ()->sign_up (string("Dummy"), this);
    }
    ImageProcessingType* get_image_processing (const std::string&, const ConfigReader&, ImageProcessingType*) throw (TribotsException,bad_alloc) {
      return new ImageProcessingDummy;
    }
  };
  Builder the_builder;
}





void ImageProcessingDummy::process_image () throw () {
  VisibleObjectList vol;
  WorldModel::get_main_world_model().set_visual_information (vol);
}
