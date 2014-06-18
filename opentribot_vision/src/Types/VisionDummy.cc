
#include "VisionDummy.h"
#include "../VisionFactory.h"
#include "../../WorldModel/WorldModel.h"

using namespace Tribots;
using namespace std;


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public VisionBuilder {
    static Builder the_builder;
  public:
    Builder () {
      VisionFactory::get_vision_factory ()->sign_up (string("Dummy"), this);
    }
    VisionType* get_vision (const std::string&, const std::string&, const ConfigReader&, VisionType*) throw (TribotsException,bad_alloc) {
      return new VisionDummy;
    }
  };
  Builder the_builder;
}





void VisionDummy::process_images () throw () {
  usleep (2000);
  VisibleObjectList vol;
  WorldModel::get_main_world_model().set_visual_information (vol, 0); // simulate source 0
}
