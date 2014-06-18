#include "ColorClasses.h"

namespace Tribots {

  ColorClassInfoList* ColorClassInfoList::theColorClassInfoList = 0;

  const ColorClassInfoList* 
  ColorClassInfoList::getColorClassInfoList(const ConfigReader* config)
  {
    if (!theColorClassInfoList) {
      if (config) {
	theColorClassInfoList = new ColorClassInfoList(*config);
      }
      else {
	theColorClassInfoList = new ColorClassInfoList();
      }
    }
    return theColorClassInfoList;
  }
  
  ColorClassInfo:: ColorClassInfo(int id, 
				  std::string name, 
				  const RGBTuple& color,
				  bool findTransitions, 
				  bool findPoints,
				  bool findOnlyFirst)
    : id(id), name(name), color(color), findTransitions(findTransitions), findOnlyFirst(findOnlyFirst),findPoints(findPoints)
  {}

  ColorClassInfoList::ColorClassInfoList()
  {
    std_init ();
  }

  ColorClassInfoList::~ColorClassInfoList()
  {
    for (unsigned int i=0; i < classList.size(); i++) {
      delete classList[i];
    }
    classList.clear();
  }

  ColorClassInfoList::ColorClassInfoList(const ConfigReader&)
  {
    std_init ();
  }

  void ColorClassInfoList::std_init()
  {
    RGBTuple black  = {   0,   0,   0 };
    RGBTuple white  = { 125, 217, 244 };
    RGBTuple grey   = { 150, 150, 150 };
    RGBTuple red    = { 159,   82,   140 };
    RGBTuple green  = {   89, 188,   154 };
    RGBTuple blue   = {   95,   133, 255 };
    RGBTuple yellow = { 203, 124,  0 };

    RGBTuple cyan   = {  50,  50,  50 };
    RGBTuple magenta= {  50,  50,  50 };

    // create color infos according to COLOR_ enum:
    classList.resize(9);
    
    // CLASS: TRANSITIONS, POINTS

    // IGNORE: no, no
    classList[COLOR_IGNORE]= new ColorClassInfo(COLOR_IGNORE,
						"IGNORE", 
						black,
						false, false);    
    // LINE: yes, no
    classList[COLOR_LINE]  = new ColorClassInfo(COLOR_LINE,
						"LINE", 
						white,
						true, false);    
    // BALL: no, yes
    classList[COLOR_BALL]  = new ColorClassInfo(COLOR_BALL,
						"BALL", 
						red,
						false, true, true);    
    // FIELD: no, no
    classList[COLOR_FIELD] = new ColorClassInfo(COLOR_FIELD,
						"FIELD", 
						green,
						false, false);    
    // OBSTACLE: no, no
    classList[COLOR_OBSTACLE] = new ColorClassInfo(COLOR_OBSTACLE,
						   "OBSTACLE", 
						   grey,
						   true, false, true);    
    // BLUE: no, no
    classList[COLOR_BLUE]  = new ColorClassInfo(COLOR_BLUE,
						"BLUE", 
						blue,
						false, false);    
    // YELLOW: no, no
    classList[COLOR_YELLOW]= new ColorClassInfo(COLOR_YELLOW,
						"YELLOW", 
						yellow,
						false, false);    

    // Next two classes are added for compatibility reasons only (
    // want to be compatible to Stephan Welkers Tool at the moment)

    // MAGENTA: no, no
    classList[COLOR_MAGENTA]=new ColorClassInfo(COLOR_MAGENTA,
						"MAGENTA", 
						magenta,
						false, false);    

    // CYAN: no, no
    classList[COLOR_CYAN]  = new ColorClassInfo(COLOR_CYAN,
						"cyan", 
						cyan,
						false, false);    

  }
    

}
