
#include "FileVision.h"
#include "../VisionFactory.h"
#include "../../Structures/Journal.h"
#include "../../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace std;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public VisionBuilder {
    static Builder the_builder;
  public:
    Builder () {
      VisionFactory::get_vision_factory ()->sign_up (string("File"), this);
    }
    VisionType* get_vision (const std::string&, const std::string&, const ConfigReader& reader, VisionType*) throw (TribotsException,bad_alloc) {
      return new FileVision (reader);
    }
  };
  Builder the_builder;
}





FileVision::~FileVision () throw () {
  if (reader)
    delete reader;
  if (stream)
    delete stream;
}

FileVision::FileVision (const ConfigReader& read) throw (std::bad_alloc, Tribots::InvalidConfigurationException) : stream(NULL), reader(NULL), cut_outside_field(false) {
  string fname;
  if (read.get ("read_world_model_info", fname)>0) {
    fname+=".vis";
    stream = new ifstream (fname.c_str());
    if (!(*stream)) {
      JERROR ("Visual info file: file error");
      delete stream;
      stream=NULL;
    } else {
      reader = new VisibleObjectReader (*stream);
    }
  }
  read.get ("FileVision::cut_lines_outside_field", cut_outside_field);
}

void FileVision::process_images () throw () {
  Time nowBegin;
  Time now;
  if (reader) {
    unsigned long int t1, t2;
    vector<VisibleObjectList> obj;
    bool success = reader->read_until (t1, t2, obj, now.get_msec());
    while (!success && nowBegin.elapsed_msec()<5000) {
      usleep (1000);
      now.update();
      success = reader->read_until (t1, t2, obj, now.get_msec());
    }
    }
}
