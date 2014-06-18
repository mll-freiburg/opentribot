
#include "Vision.h"
#include "VisionType.h"
#include "VisionFactory.h"
#include "../Structures/Journal.h"
#include "cstring"

using namespace Tribots;
using namespace std;

const char* Tribots::Vision::get_vision_type () const throw () { return vision_descriptor; }

Tribots::Vision::~Vision () throw () {
  delete the_vision;
  delete [] vision_descriptor;
}

Tribots::Vision::Vision (const ConfigReader& vread) throw (TribotsException, bad_alloc) : the_vision(NULL), configuration_list(vread) {
  string confline;
  string section;
  cout << "Tribots vision"<<endl;
  if (vread.get("vision_type", confline)<=0) {
    JERROR("no config line \"vision_type\" found");
    throw Tribots::InvalidConfigurationException ("vision_type");
  }
  if (vread.get("vision_section", section) <=0) {
    JERROR("no config line \"vision_section\" found");
    throw Tribots::InvalidConfigurationException("vision_section");
  }
  try{
    really_change_vision_type (confline.c_str(), section.c_str(), vread);
  }catch(TribotsException& e){
    JERROR((std::string ("creating vision of type ")+confline+std::string(" failed due to: ")+std::string(e.what())).c_str());
    throw (e);
  }
}

bool Tribots::Vision::change_vision_type (const char* ipt, const char* section) throw () {
  return change_vision_type (ipt, section, configuration_list);
}

bool Tribots::Vision::change_vision_type (const char* ipt, const char* section, const ConfigReader& vread) throw () {
  try{
    really_change_vision_type (ipt, section, vread);
  }catch(bad_alloc&){
    JWARNING("Change of vision type failed due to lack of memory");
    return false;
  }catch(TribotsException&){
    JWARNING("Change of vision type failed");
    return false;
  }
  return true;
}

void Tribots::Vision::really_change_vision_type (const char* ipt, const char* section, const ConfigReader& vread) throw (TribotsException, bad_alloc) {
  VisionType* new_vision;
  char* new_descriptor;
  try{
    string ipts (ipt);
    new_vision=VisionFactory::get_vision_factory()->get_vision(ipts, section, vread);
  }catch(invalid_argument&){
    throw Tribots::InvalidConfigurationException ("vision_type");
  }
  
  new_descriptor = new char [strlen(ipt)+1];
  strcpy(new_descriptor,ipt);
  if (the_vision!=NULL) {
    delete the_vision;
    delete [] vision_descriptor;
  }
  the_vision=new_vision;
  vision_descriptor=new_descriptor;
}

