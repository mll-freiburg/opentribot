
#include "FileVision.h"
#include "../VisionFactory.h"
#include "../../Structures/Journal.h"
#include "../../WorldModel/WorldModel.h"
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
    if (success) {
      if (cut_outside_field) {
        double w = 0.5*MWM.get_field_geometry ().field_width+MWM.get_field_geometry().side_band_width;
        double l = 0.5*MWM.get_field_geometry ().field_length+MWM.get_field_geometry().goal_band_width;
        for (unsigned int i=0; i<obj.size(); i++) {
          RobotLocation rloc = MWM.get_robot_location (obj[i].timestamp);
          vector<VisibleObject>::iterator it = obj[i].objectlist.begin();
          while (it < obj[i].objectlist.end()) {
            Vec p = rloc.pos+(it->pos*rloc.heading);
            if (p.y<-l || l<p.y || p.x<-w || w<p.x)
              it = obj[i].objectlist.erase (it);
            else
              it++;
          }
        }
      }
      for (unsigned int i=0; i<obj.size(); i++)
        MWM.set_visual_information (obj[i], i);
    }
  }
}
