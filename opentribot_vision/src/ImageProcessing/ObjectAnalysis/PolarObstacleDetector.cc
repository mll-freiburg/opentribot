#include "PolarObstacleDetector.h"
#include "ColorClasses.h"
#include "../Formation/Painter.h"
#include <cmath>

namespace Tribots {
   using namespace std;

   /** constructor
    */
   PolarObstacleDetector::PolarObstacleDetector(
            const ConfigReader& config,
			   const ImageWorldMapping* mapping, 
			   const WorldImageMapping* rel2img,
			   double minTransitionSize) 
      : ObstacleDetector(), mapping(mapping), rel2img(rel2img),
         minTransitionSize(minTransitionSize), vis(0) {
    
   //if (config.get((configSection+"::obstacle_detector").c_str(), obstDectName) < 0) {
   //  throw InvalidConfigurationException("ScanLine::obstacle_detector");
   //}   
  
    pcluster = new PolarObstacleCluster(config, 180 );
   } 
  

   /** destructor
    */
   PolarObstacleDetector::~PolarObstacleDetector() {
      delete pcluster;
   }
  

   /** method searchObstacles
    */
   void PolarObstacleDetector::searchObstacles(
               const Image&, 
               const ScanResult* scanResult, Time time,
               VisibleObjectList* vol)
                  throw (TribotsException) {
      
      if (scanResult->id != COLOR_OBSTACLE) {
         throw TribotsException(__FILE__ 
			       ": Expected results of color class "
			       "COLOR_OBSTACLE but received results of another "
			       "class.");
      }
   
      // ----- Convert image pixel transitions to relative robot points -----
      std::vector<Vec>  opoints; // the transitions obstacle points in relative robot coordinates
      Vec               obstaclePosition;
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
      
         //if (transStart.virt == true || transEnd.virt == true) { // skip virtual
         //continue;
         //}
      
         // plausibilitätstest, minimale Breite erforderlich
         if ((transStart.toPos - transEnd.fromPos).length() < minTransitionSize) {
	         continue;
         }

         // Plausibilitätstest ... angesetzte Scanlnien können im Hindernis beginnen ... nicht ganz sauber
         //if ((mapping->map(transStart.toPos)).length() > 1000.0 && transStart.virt == true) continue;

         // Position berechnen; \todo nicht einfach den ersten Punkt nehmen
         obstaclePosition = mapping->map(transStart.toPos);      
         opoints.push_back(obstaclePosition);
      }

      // ----- Now cluster points -----
      pcluster->cluster(opoints);
      pcluster->vis();

      std::vector< Obstacle > & obs = pcluster->obstacles;
      for (unsigned int i=0; i<obs.size();i++)
         vol->objectlist.push_back( VisibleObject (obs[i].support,
                                                   VisibleObject::obstacle,
                                                   obs[i].width) );
      if (vis) {
         paintResults(obs);
         vis = 0;
      }
   } // end of method searchObstacles

  
   /** method paintResults
    *
    * used as output on debug-image
    */
   void PolarObstacleDetector::paintResults(std::vector< Obstacle > & obs) {
      Painter p(*vis);
      p.setColor(200, 200, 200);

      for (unsigned int cl=0; cl < obs.size(); cl++) {
         p.setPen(cl%2);
         p.setColor(((cl/2)%2) * 50 + 150, 
          ((cl/2)%2) * 50 + 150, 
          ((cl/2)%2) * 50 + 150);

         p.setPen(Painter::PEN_SOLID);

         Vec point = rel2img->map(obs[cl].support);
         int point_x = (int) (point.x + 0.5);
         int point_y = (int) (point.y + 0.5);
         p.markRect(point_x, point_y, 3);
         p.markRect(point_x, point_y, 5);


         // draw circle
         const int circlepoints = 16;
         double radius = obs[cl].width/2;
         Vec mappedStart, mappedEnd;
         Vec end;
         Vec middle = obs[cl].support;
         Vec start = obs[cl].support;
         start.x += radius; // start at (x + radius, y)

         for (int i=1; i<=circlepoints; i++) {
            // turn around angle a
            double a = (i*2.0*M_PI)/circlepoints;
            end.x = static_cast<int>(middle.x+radius*std::cos(a));
            end.y = static_cast<int>(middle.y+radius*std::sin(a));

            // mapping and paint
            mappedStart = rel2img->map(start);
            mappedEnd = rel2img->map(end);
            
            p.drawLine ((int) (mappedStart.x + 0.5),  
                        (int) (mappedStart.y + 0.5), 
                        (int) (mappedEnd.x + 0.5), 
                        (int) (mappedEnd.y + 0.5) );

            // set new start point
            start.x=end.x;
            start.y=end.y;
         } // circle is ready

      }   
   } // end of method paintResults
};
