#include "GoalDetector.h"
#include "ColorClasses.h"
#include "../Formation/Painter.h"
#include "../../WorldModel/WorldModel.h"
#include <cstdlib>
#include <iostream>

namespace Tribots {

  using namespace std;

  GoalDetector::GoalDetector(int centerX, int centerY, 
			     const WorldImageMapping* mapping,
			     int width, int height, int n)
    : center(Vec(centerX,centerY)), width(width), height(height), n(n),
      mapping(mapping), vis(0)
  {}

  void 
  GoalDetector::searchGoals(const Image& image, Time time,
			    VisibleObjectList* vol)
    throw (TribotsException)
  {
    WorldModel& worldModel = WorldModel::get_main_world_model();
    RobotLocation location = worldModel.get_robot_location(time); 
    // muss globale weltkooridnaten des tors um -heading drehen
    
    Vec tor1 (0, worldModel.get_field_geometry().field_length/2.);
    Vec tor2 (0, worldModel.get_field_geometry().field_length/-2.);

    Vec dirV1 = (tor1 - location.pos).rotate(-location.heading); // relative
    Vec dirV2 = (tor2 - location.pos).rotate(-location.heading); // direction

    Vec tor1img;
    Vec tor2img;

    if (mapping != 0) { // if inverse mapping is available use it!
      // search 1m after crossing the goal line:
      Vec dirS1 = dirV1.normalize();    // direction, scaled
      dirS1 *= dirV1.length() + 2000.;  // + 2m 
      Vec dirS2 = dirV2.normalize();
      dirS2 *= dirV2.length() + 2000.;  // + 2m 

      tor1img = mapping->map( dirS1 );      // transform relative coords
      tor2img = mapping->map( dirS2 );      // to image coords
    }
    else {              // use an aproximation (magic value by S. Welker)
      tor1img = dirV1.normalize();
      tor1img *= 205; 
      tor2img= dirV2.normalize();
      tor2img *= 205; 

      tor1img = tor1img + center;
      tor2img = tor2img + center;
    }

    if (vis) {          // visualize search area
      Painter painter(*vis);
      painter.setColor(Painter::black);
      painter.setPen(Painter::PEN_STEPPED);
      painter.drawRect((int)(tor1img.x-width/2), 
		       (int)(tor1img.y-height/2),
		       width, height);
      painter.drawRect((int)(tor2img.x-width/2), 
		       (int)(tor2img.y-height/2),
		       width, height);
    }

    int col1 = getDominantColor(image, tor1img, n);
    int col2 = getDominantColor(image, tor2img, n);

    if (col1 != col2) {      // goal(s) found iff colors differ
      if (col1 == COLOR_BLUE) {
	if (vis) {
	  visualizeGoal(tor1img, COLOR_BLUE);
	}
   vol->objectlist.push_back(
	    VisibleObject(dirV1, VisibleObject::blue_goal)
	  );
      }
      else if (col1 == COLOR_YELLOW) {
	if (vis) {
	  visualizeGoal(tor1img, COLOR_YELLOW);
	}
	  vol->objectlist.push_back(
	    VisibleObject(dirV1, VisibleObject::yellow_goal)
	  );
      }

      if (col2 == COLOR_BLUE) {
	if (vis) {
	  visualizeGoal(tor2img, COLOR_BLUE);
	}
	  vol->objectlist.push_back(
	    VisibleObject(dirV2, VisibleObject::blue_goal)
	  );
      }
      else if (col2 == COLOR_YELLOW) {
	if (vis) {
	  visualizeGoal(tor2img, COLOR_YELLOW);
	}
	  vol->objectlist.push_back(
	    VisibleObject(dirV2, VisibleObject::yellow_goal)
	  );
      }
    }
    if (vis) vis = 0;       // finished visualization
  }

  void
  GoalDetector::visualizeGoal(const Vec& pos, int col)
  {
    Painter p(*vis);
    p.setColor(ColorClassInfoList::getColorClassInfoList()->classList[col]->color);
    p.drawRect((int)pos.x-width/2-1, (int)pos.y-height/2-1, width+2, height+2);
    p.drawRect((int)pos.x-width/2-2, (int)pos.y-height/2-2, width+4, height+4);
    p.drawRect((int)pos.x-width/2-3, (int)pos.y-height/2-3, width+6, height+6);
    p.drawRect((int)pos.x-width/2-4, (int)pos.y-height/2-4, width+8, height+8);
  }

  int
  GoalDetector::getDominantColor(const Image& image, const Vec& position, 
				 int n)
  {
    int c = 0;
    int blue = 0;
    int yellow = 0;

    // n Pixel um die Position streuen, Anzahl von blau und gelb 
    // zählen:

    Painter* painter=0;
    if (vis) painter = new Painter(*vis);

    for (int i=0; i < n; i++) {     // loop draws n pixels
      Vec r(static_cast<int>((double)width*rand()/(RAND_MAX+1.0)) - (width/2),
	    static_cast<int>((double)height*rand()/(RAND_MAX+1.0)) - (height/2));

      r += position;                // calculate pixel position

      r.x = min(max(r.x, 0.), image.getWidth() - 1.);  // clip at boundaries
      r.y = min(max(r.y, 0.), image.getHeight() - 1.); // clip at boundaries

      c = image.getPixelClass( static_cast<int>(r.x), static_cast<int>(r.y) );

      if (vis) {
	painter->setColor(ColorClassInfoList::getColorClassInfoList()->classList[c]->color);
	painter->markCrossbar((int)r.x, (int)r.y, 2);
      }
	

      if ( c == COLOR_BLUE ) {
	blue++;
      }
      else if ( c == COLOR_YELLOW ) {
	yellow++;
      }
    }
    if (vis) delete painter;

    if ( yellow > n * 0.25 && blue < n * 0.1 ) {
      return COLOR_YELLOW;
    }
    else if (blue > n * 0.25 && yellow < n * 0.1 ) {
      return COLOR_BLUE;
    }
    else {
      return COLOR_IGNORE;
    } 
  }
}
