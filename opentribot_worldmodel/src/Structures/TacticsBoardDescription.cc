
#include "TacticsBoardDescription.h"

using namespace Tribots;
using namespace std;

void Tribots::read_tactics (std::vector<TacticsAttribute>& dest, const ConfigReader& cfg) throw (std::bad_alloc) {
  std::vector<std::string> attributelist, optionlist;
  cfg.get ("Coach::tactics_attributes", attributelist);
  for (unsigned int i=0; i<attributelist.size(); i++) {
    TacticsAttribute attr;
    attr.name = attributelist[i];
    if (attr.name.length()>0 && attr.name[0]=='*') {
      attr.editable=true;
      attr.name=attr.name.substr(1,attr.name.length()-1);
    } else {
      attr.editable=false;
    }
    attr.force_default=false;
    cfg.get ((string("Coach::")+attr.name).c_str(), optionlist);
    bool force_found=false;
    if (optionlist.size()>0) {
      unsigned int default_value_index=0;
      for (unsigned int j=0; j<optionlist.size(); j++)  {
        if (optionlist[j][0]=='*') {
          if (!force_found)
            default_value_index=j;
          attr.options.push_back (optionlist[j].substr(1,optionlist[j].length()));
        } else if (optionlist[j][0]=='+') {
          default_value_index=j;
          attr.options.push_back (optionlist[j].substr(1,optionlist[j].length()));
          attr.force_default=true;
          force_found=true;
        } else {
          attr.options.push_back (optionlist[j]);
        }
      }
      if (default_value_index<attr.options.size())
        attr.default_value = attr.options[default_value_index];
      else
        attr.default_value = "";
      dest.push_back (attr);
    }
  }
}

void Tribots::make_default_tactics_board (TacticsBoard& dest, std::vector<TacticsAttribute>& src, bool clear) throw (std::bad_alloc) {
  if (clear) {
    dest.clear();
  }
  for (unsigned int i=0; i<src.size(); i++) {
    dest[src[i].name]=src[i].default_value;
  }
}

/*
void Tribots::update_tactics_board_selected (TacticsBoard& dest, const TacticsBoard& src) throw (std::bad_alloc) {
  std::map<std::string, strd::string>::const_iterator srcit = src.begin();
  while (srcit!=src.end()) {
    dest[srcit->first] = srcit->second;
    srcit++;
  }
}
*/
