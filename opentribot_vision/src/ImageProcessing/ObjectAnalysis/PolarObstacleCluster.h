#ifndef _tribots_polarobstaclecluster_h_
#define _tribots_polarobstaclecluster_h_

#include "../../Fundamental/ConfigReader.h"
#include "../../Fundamental/Vec.h"
#include "../../Fundamental/Time.h"
#include <vector>

namespace Tribots {

   struct s_polarIdClust {
      int idvon;
      int idbis;
   };


   /** Repräsentation eines Hindernisses */
   class Obstacle {
      public:
         /** Standardkonstruktor */
         Obstacle (Vec, double);

         // Position des Hindernisses
         Vec support;                      

         // Breite des Hindernisses in mm
         double width;                     

   };


  class PolarObstacleCluster {

      public:
         PolarObstacleCluster(const ConfigReader& cfg, 
                              int polarresolution);
         ~PolarObstacleCluster();
       
         void cluster(const std::vector< Vec >& opoints);
       
         void vis();

         // all obstacles are stored here
         std::vector< Obstacle > obstacles;                    


      protected:
         /* some configuration is following*/

         // squared max distance 
         // for obstacles (in m)
         static const int squaredMaxObstacleDist = 6*6;       
                                                            
         // maximaler Abstand zwischen 
         // erkannten Hindernissen
         // um ein Obstacle zu sein 
         static const double maxRelativeDistance = 0.5*0.5;   
                                                            
         // max points in cluster which may be no obstacle
         static const int maxFailed =  4;

         // obsolete
         // how many segments must a cluster have?
         static const int minClusterSize = 2;	               

         // better:
         // Mindest Scangröße (in m)
         static const double minObstacleWidth = 0.1;          

         /* end of configuration section */


         // size of cluster array
         int    resolution;
         double dangle;

         // relation table: 
         // index (for polarCluster) refers 
         // point from vector opoints
         int* opointsRelation;                                
                                                            
         // main tool for clustering:
         // put all distances in here, depending on there angle
         double *polarDistanceBuffer;
         std::vector< s_polarIdClust > polarCluster;

         // pointer to original points -> 
         // at first add something and then cluster
         const std::vector< Vec >* origPoints;                
                                                            
         // for timemeasuring purposes
         Time debugTime;


         // functions
         void add(const std::vector< Vec >& opoints);
         void add(const Vec& opoint, int opointIndex);
         void clear();
         bool isRecognizableObject(double previousDistance, 
                                    double distance); 
         void finishCluster(int begin, int end, 
                              double mindist);
   };

}

#endif
