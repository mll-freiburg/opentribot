
#include "WorldModel.h"
#include "WorldModelFactory.h"
#include "../Structures/Journal.h"
#include <cstring>

using namespace Tribots;
using namespace std;

Tribots::WorldModel* Tribots::WorldModel::main_world_model=NULL;

Tribots::WorldModel& Tribots::WorldModel::get_main_world_model () throw () { return *Tribots::WorldModel::main_world_model; }


bool Tribots::WorldModel::is_main_world_model_available () throw()
{
  return main_world_model != NULL;
}

const char* Tribots::WorldModel::get_world_model_type () const throw () { return world_model_descriptor; }

Tribots::WorldModel::~WorldModel () throw () {
  if (Tribots::WorldModel::main_world_model==this)
    Tribots::WorldModel::main_world_model=NULL;    // aus Sicherheitsgruenden
  delete the_world_model;
  delete [] world_model_descriptor;
}

Tribots::WorldModel::WorldModel (const ConfigReader& vread) throw (TribotsException,bad_alloc) : the_world_model(NULL), configuration_list(vread) {
  if (Tribots::WorldModel::main_world_model==NULL) 
    Tribots::WorldModel::main_world_model=this;   // erste Instanziierung
  string confline;
  if (vread.get("world_model_type", confline)<=0) {
    JERROR("no config line \"world_model_type\" found");
    throw Tribots::InvalidConfigurationException ("world_model_type");
  }

    cout<< "Initializing Worldmodel : "<<confline.c_str()<<endl;

  try{
    really_change_world_model_type (confline.c_str(), vread);
  }catch(TribotsException& e){
    JERROR((std::string ("creating world model of type ")+confline+std::string(" failed due to: ")+std::string(e.what())).c_str());
    throw (e);
  }
}

bool Tribots::WorldModel::change_world_model_type (const char* wmt) throw () {
  return change_world_model_type (wmt, configuration_list);
}

bool Tribots::WorldModel::change_world_model_type (const char* wmt, const ConfigReader& vread) throw () {
  try{
    really_change_world_model_type (wmt, vread);
  }catch(bad_alloc&){
    JWARNING("Change of world model type failed due to lack of memory");
    return false;
  }catch(TribotsException&){
    JWARNING("Change of world model type failed");
    return false;
  }
  return true;
}

void Tribots::WorldModel::really_change_world_model_type (const char* wmt, const ConfigReader& reader) throw (TribotsException,bad_alloc) {
  WorldModelType* new_world_model;
  char* new_world_model_descriptor;
  try {
    string wmts (wmt);
    new_world_model=WorldModelFactory::get_world_model_factory ()->get_world_model (wmts, reader);
    cout <<"WorldModelTypeString: "<<wmts<<endl;


  }catch (invalid_argument){
    throw Tribots::InvalidConfigurationException ("world_model_type");
  }
  
  new_world_model_descriptor=new char [strlen(wmt)+1];
  strcpy (new_world_model_descriptor, wmt);
  if (the_world_model!=NULL) {
    delete the_world_model;
    delete [] world_model_descriptor;
  }
  the_world_model=new_world_model;
  world_model_descriptor=new_world_model_descriptor;
}
