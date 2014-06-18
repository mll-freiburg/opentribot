#include "LineDetector.h"
#include "ColorClasses.h"
#include "../Formation/Painter.h"
#include <cmath>


namespace Tribots {

  LineDetector::LineDetector(const ImageWorldMapping* mapping, 
			     FieldMapper * fieldmapper,
			     double minLineWidth,
			     double maxLineWidth,
			     bool use2Steps,
			     bool checkForGreen,
			     bool cutOutsideField,
			     bool useMaximalDistance,
			     double maximalDistance,
			     bool useHalfField,
			     bool useFieldMapper) 
    : mapping(mapping),fieldmapper(fieldmapper), minLineWidth(minLineWidth), maxLineWidth(maxLineWidth),
      use2Steps(use2Steps), checkForGreen(checkForGreen), 
      cutOutsideField(cutOutsideField), useHalfField(useHalfField),
      useMaximalDistance(useMaximalDistance), maximalDistance(maximalDistance),useFieldMapper(useFieldMapper), vis(0)
  {
  }
  
  void
  LineDetector::searchLines(const ScanResult* scanResult, Time time, 
			    VisibleObjectList* vol)
    throw (TribotsException)
  {
    if (scanResult->id != COLOR_LINE) {
      throw TribotsException(__FILE__ 
			     ": Expected results of color class COLOR_LINE "
			     "but received results of another class.");
    }
    
    for (unsigned int i=0; i < scanResult->transitions.size(); i+=2) {
      if (scanResult->transitions.size() <= i+1) { // a pair left?
	throw TribotsException(__FILE__
			       ": The transition list is not organized "
			       "pair-wise. One end transision is missng.");
      }
	
      const Transition& transStart = scanResult->transitions[i];
      const Transition& transEnd   = scanResult->transitions[i+1];

      if (transStart.type != Transition::START ||
	  transEnd.type   != Transition::END) {
	throw TribotsException(__FILE__
			       ": The transition list is not organized "
			       "pair-wise. Each start transtition has to be "
			       "followed by exactly one endtransision!");
      }

      // plausibility check
     // RobotLocation robotLocation = MWM.get_robot_location(time);
      Frame2d robot2world;
      robot2world.set_position (Vec(0,0));
      robot2world.set_angle (Angle(0));

      if (checkLinePlausibility(transStart, transEnd, 
				robot2world)) { // add to world model?
	Vec middle = (mapping->map(transStart.toPos) +
		      mapping->map(transEnd.fromPos));
	middle *= 0.5;  // Mittelpunkt auf der Linie zwischen den beiden 
	                // Übergängen
	if (vis) {
	  Vec imgMiddle = (transStart.toPos + transEnd.fromPos) * .5;
	  visualize(imgMiddle);
	}
	
  vol->objectlist.push_back(
   VisibleObject (middle, VisibleObject::white_line)
 );
      }
    }
    if (vis) vis = 0;  // visualization finished
  }

  bool 
  LineDetector::checkLinePlausibility(const Transition& transStart,
				      const Transition& transEnd,
				      const Frame2d& robot2world) const
  {
    if (checkForGreen) {  // check if FIELD before and after line
      if (use2Steps) {
	if (transStart.twoStep != COLOR_FIELD ||
	    transEnd.twoStep != COLOR_FIELD) {
	  return false;
	}
      }
      else if (transStart.from != COLOR_FIELD ||
	       transEnd.to != COLOR_FIELD) { 
	return false;
      }
    }
    else {  // check for same color class
      if (use2Steps) {
	if (transStart.twoStep != transEnd.twoStep) {
	  return false;
	}
      }
      else if (transStart.from != transEnd.to) { 
	// same color before and after line
	return false;
      }
    }
    if (useMaximalDistance) {
      Vec globalCoord = robot2world * mapping->map(transStart.toPos);

      if (globalCoord.length()  > maximalDistance) {
	return false;
      }
    }

    if (cutOutsideField) {
//      const FieldGeometry& fg = MWM.get_field_geometry ();
      Vec globalCoord = Vec(0,0);//robot2world * mapping->map(transStart.toPos);
      
      double maxX = 15000;
      double maxY = 20000;

      if (fabs(globalCoord.x) > maxX || fabs(globalCoord.y) > maxY) {
	return false;
      }
    }      

    
    if (useFieldMapper) {  // nur halbes Feld vorhanden (Trainingsraum)
      if (! fieldmapper->insideField((int)transStart.fromPos.x,
				     (int)transStart.fromPos.y))
      //std::cout<<"FieldMapper Check For "<<transStart.fromPos <<"\n";
      return false;
      }
    
      

    double width = ( mapping->map(transStart.toPos) - 
		     mapping->map(transEnd.fromPos) ).length();

    if (width < minLineWidth || // check distance between the two transitions
	width > maxLineWidth) {
      return false;
    }

    return true;                // passed all tests
  }

  void LineDetector::visualize(const Vec& line) const
  {
    Painter p(*vis);
    p.setColor(60, 120, 240);
    p.markRect((int)line.x, (int)line.y, 2);
  }
};
