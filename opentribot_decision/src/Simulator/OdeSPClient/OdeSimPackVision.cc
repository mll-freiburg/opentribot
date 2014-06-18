
#include "OdeSimPackVision.h"
#include "../../ImageProcessing/VisionFactory.h"
#include "../../WorldModel/WorldModel.h"
#include "../../Structures/Journal.h"

using namespace Tribots;
using namespace std;


// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public VisionBuilder {
    static Builder the_builder;
  public:
    Builder () {
      VisionFactory::get_vision_factory ()->sign_up (string("OdeSimPackVision"), this);
    }
    VisionType* get_vision (const std::string&, const std::string&, const ConfigReader& cfg, VisionType*) throw (TribotsException,bad_alloc) {
      return new OdeSimPackVision(cfg);
    }
  };
  Builder the_builder;
}

OdeSimPackVision::OdeSimPackVision(const ConfigReader& cfg) throw (Tribots::TribotsException)
{
  client = 0;
  client = OdeSimPackClient::getTheClient(&cfg);
}

void OdeSimPackVision::process_images () throw (Tribots::BadHardwareException) {
 try {
    client->receiveData(); // wartet bis Simulation neue daten liefert und speichert diese im client
  }
  catch (Tribots::HardwareException &e) {
    JWARNING(e.what());
  }
}
