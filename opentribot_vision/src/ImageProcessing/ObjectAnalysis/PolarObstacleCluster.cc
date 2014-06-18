#include "PolarObstacleCluster.h"
#include <cmath>
#include "../../Fundamental/Frame2D.h"

//#define DEBUG_OBS_CLUSTER

using namespace Tribots;
using namespace std;


// implement Obstacle-constructor
Obstacle::Obstacle (Vec support, double width):
                        support(support), width(width) {;}


/** constructor
 *
 * @param cfg ConfigReader
 * @param polarresolution how many slices has our circle
 */
PolarObstacleCluster::PolarObstacleCluster(const ConfigReader& cfg, 
                                             int polarresolution)
                                       : resolution( polarresolution ) 
{
  polarDistanceBuffer = new double[resolution];
  dangle              = (2*M_PI) / resolution;
  opointsRelation     = new int[resolution];
  this->clear();
}


/** destructor
 * 
 */
PolarObstacleCluster::~PolarObstacleCluster()
{
  delete[] polarDistanceBuffer;
  delete[] opointsRelation;
}


/** method void clear()
 *
 *  initializes the intern polarDistanceBuffer
 */
void PolarObstacleCluster::clear() {
   for (int i=0; i<resolution; i++) {
      polarDistanceBuffer[i] = -1;
      opointsRelation[i] = -1;
   }
   polarCluster.clear();
   obstacles.clear();
}


/** method add( vector< Vec > )
 *
 * calls add for each point with correct index
 *
 * @param opoints original points to cluster
 *
 */
void PolarObstacleCluster::add(const std::vector< Vec >& opoints) {
  this->origPoints = &opoints;
  for (unsigned int i=0; i<opoints.size(); i++) add(opoints[i], i);
}


/** method add( vector, int )
 *
 * inserts a new point with its index
 *
 * @param opoint original point to cluster
 * @param opointsIndex index of Originalpoint in originalVecArray, important for getting it later back from opointsRelation
 */
void PolarObstacleCluster::add(const Vec& opoint, int opointsIndex) {
  double ang   = atan2(opoint.y,opoint.x);
  if (ang < 0) ang += 2*M_PI;
  int    id    = ((int)floor(ang/dangle)) % resolution;
  double sdist = (opoint.x / 1000.0 * opoint.x / 1000.0) + 
                  (opoint.y / 1000.0 * opoint.y / 1000.0);
  
  //LOUT << " ID for: " << opoint.x << " " << opoint.y << " : " 
  //     << id << " SDIST: " << sdist << "\n";

   if ((sdist < squaredMaxObstacleDist ) && 
       (polarDistanceBuffer[id] == -1 || sdist < polarDistanceBuffer[id])) 
   {
      polarDistanceBuffer[id] = sdist;
      opointsRelation[id] = opointsIndex; // set relation
   }
}


/** Method cluster() does the dirty work
 *
 * @param opoints original found points to get clustered
 */
void PolarObstacleCluster::cluster(const std::vector< Vec >& opoints) {


  // init cluster-array
  this->clear(); 

  // add all originalpoints into array of polar coordinates
  add(opoints);

  // find entry point for the clustering to avoid clusteringbounds
  int entry; 
  int noObject=0;
  for (entry=0; (entry<resolution) && (noObject < maxFailed); entry++) {
        // entry must be no valueable object
        if (!isRecognizableObject(polarDistanceBuffer[entry],  // last value is not important
                                    polarDistanceBuffer[entry]))
		      noObject++;
        else
            noObject=0;
  }

#ifdef DEBUG_OBS_CLUSTER
  LOUT << "found entry for clustering at position " << entry << endl;
  debugTime.update();
#endif

  int clusterBegin    = -1;
  int lastPos 	      = -1;
  int pos             =  0;
  double lastValue    =  0;
  int failed          =  0;
  // min distance important for finishCluster
  double minDistance = squaredMaxObstacleDist; 

  // scan entire circle
  //
  for (int i=entry; i<resolution+entry; i++) {

      pos = i % resolution; // prevent array-out-of-bounds-error

#ifdef DEBUG_OBS_CLUSTER
	   LOUT << "polarDistanceBuffer[" << pos << "]: " 
		<< polarDistanceBuffer[pos] << endl;
#endif

      // check lastValue of the last part of this cluster
      if ((-1 == lastPos) || (lastPos + maxFailed < pos))
	      lastValue = polarDistanceBuffer[pos];
      else
	      lastValue = polarDistanceBuffer[lastPos];

      if (isRecognizableObject(lastValue, polarDistanceBuffer[pos])) { 
         // found a object !
	      failed = 0;
	      lastPos = pos; 		// store this position
         if (minDistance > polarDistanceBuffer[pos]) // update minimal
            minDistance = polarDistanceBuffer[pos];  // distance

	      if ( clusterBegin < 0 ) {
	         clusterBegin = pos;

#ifdef DEBUG_OBS_CLUSTER
	         LOUT << "Begin cluster at " << pos << "\n";
#endif

	      }
	} else {
	   failed++;
	   if ((failed >= maxFailed) && (clusterBegin >= 0)){  // end of this cluster reached
                int end = pos-failed;
                if (end < 0) // modulo resolution
                  end += resolution;
                int size = end - clusterBegin + 1;
                if (size < 0) // modulo resolution
                  size += resolution;

#ifdef DEBUG_OBS_CLUSTER
      LOUT << "End cluster at " << end << "\n";
      LOUT << "size of cluster: " << size << " segments\n";
#endif

      // conditional from size
	   if (size >= minClusterSize)
			finishCluster(clusterBegin, end, minDistance);
		else {
         ;
#ifdef DEBUG_OBS_CLUSTER
			LOUT << "cluster is to small" << "\n";
#endif
      }

		// reinit values
                minDistance = squaredMaxObstacleDist; 
		clusterBegin = -1;
		lastPos = -1;
		// reinit clustering pointer for position after finished cluster
		i -= failed;
	    }
	}
    }// end of scan entire circle
  // do final check
  //

   if (clusterBegin >= 0){  // cluster was begon, but not finished
      int end = pos-failed;
      if (end < 0) // modulo resolution
        end += resolution;
      int size = end - clusterBegin + 1;
      if (size < 0) // modulo resolution
        size += resolution;

#ifdef DEBUG_OBS_CLUSTER
      LOUT << "not finished, now forcing finish" << "\n";
      LOUT << "End cluster at " << end << "\n";
      LOUT << "size of cluster: " << size << " segments\n";
#endif

      // conditional from size
      if (size >= minClusterSize)
        finishCluster(clusterBegin, end, minDistance);
      else {
         ;
#ifdef DEBUG_OBS_CLUSTER
			LOUT << "cluster is to small" << "\n";
#endif
      }
  }
  // end of final check

#ifdef DEBUG_OBS_CLUSTER
  LOUT << "clustering points needs: " << debugTime.elapsed_usec() 
       << " microseconds.\n";
#endif

} // end of cluster



/** we have recognized a cluster, thus store the cluster
 *
 * @param begin polarID for ClusterBegin
 * @param end polarID for ClusterEnd
 * @param mindist squared minimal distance
 */
void PolarObstacleCluster::finishCluster(int begin, int end, double mindist) {
    s_polarIdClust myCluster;
    myCluster.idvon = begin;
    myCluster.idbis = end;
    polarCluster.push_back(myCluster);

    int vecIndex;

    // calc middle of Obstacle
    // Left
    vecIndex = opointsRelation[begin];
    // complex pointer dereferencing syntax
    const Vec& vecLeft = (*origPoints)[vecIndex]; 
    // Right
    vecIndex = opointsRelation[end];
    // complex pointer dereferencing syntax
    const Vec& vecRight = (*origPoints)[vecIndex]; 

    // real middle
    Vec diffVec = vecRight - vecLeft;
    Vec realMiddle = vecLeft + 0.5*diffVec;
    // calculate width
    double width = diffVec.length();
/*
    // do not recognize this obstacle if to small
    if (minObstacleWidth > width) {
#ifdef DEBUG_OBS_CLUSTER
    LOUT << "Obstacle with width " << width << " is to small.\n";
#endif
       return;
    }
*/

    // obstacle should begin at its nearest cluster -> move realMiddle
    // nearest cluster should be found during clustering
    mindist = sqrt(mindist)*1000; // in mm
    realMiddle = mindist*realMiddle.normalize();

    // insert obstacle
    Obstacle ob(realMiddle, width);
    obstacles.push_back(ob);

#ifdef DEBUG_OBS_CLUSTER
    LOUT << "Cluster found!\n";
    LOUT << "real Middle: " << realMiddle.x  << " " << realMiddle.y << "\n";
    LOUT << "width: " << width << std::endl;
    LOUT << "mindist: " << mindist << std::endl;
    // visualize middle
    Time now;
    RobotLocation rl = MWM.get_robot_location(now);
    Frame2d robot2world;
    robot2world.set_position(rl.pos);
    robot2world.set_angle(rl.heading);
    Vec absop = robot2world*(realMiddle); // convert coord
    LOUT << "\% red circle " << absop.x << "  " << absop.y << " " << (int)width << " \n";
#endif
}


/** check if a distance should be clustered to a previous distance
 *
 * @param previousDistance which was the distance in the previous polar segment
 * @param distance current polar segments distance
 *
 * @return true if it could be appended, else false
 */
bool PolarObstacleCluster::isRecognizableObject(double previousDistance, double distance) { 

      // obstacle must be recognized and within maxObstacleDist and must
      // be "connected" to the previous distance, thus there must not be a large jump

        if (distance < 0)
                return false;


        if (abs(sqrt(previousDistance)-sqrt(distance)) > sqrt(maxRelativeDistance)) {
#ifdef DEBUG_OBS_CLUSTER
         LOUT << "distance of segment before this segment in current cluster " << previousDistance << "\n";
		   LOUT << "distance " << distance << "\n";
         LOUT << "relative distance " << abs(previousDistance-distance) << " to far\n";
#endif
         return false;
        }
        return true;
}


void PolarObstacleCluster::vis() {
#ifdef DEBUG_OBS_CLUSTER
  Time now;
  RobotLocation rl = MWM.get_robot_location(now);
  Frame2d robot2world;
  robot2world.set_position(rl.pos);
  robot2world.set_angle(rl.heading); 

  for (int i=0; i<resolution; i++) {
      if (polarDistanceBuffer[i] > 0) {
	Vec relop = Vec(sqrt(polarDistanceBuffer[i]) * 1000.0, 0) * Angle(dangle * i);
	Vec absop = robot2world*(relop);
	LOUT << "\% black circle " << absop.x << "  " << absop.y << " 100 \n";
	LOUT << "\% black line " 
	     << rl.pos.x << "  " << rl.pos.y << "  "
	     <<  absop.x << "  " << absop.y << "\n";
	  
      } else {
	Vec relop = Vec(1000.0, 0) * Angle(dangle * i);
	Vec absop = robot2world*(relop);
	//LOUT << "\% blue circle " << absop.x << "  " << absop.y << " 50 \n"; 
//LOUT << "\% blue thin line " 
//     << rl.pos.x << "  " << rl.pos.y << "  "
//     <<  absop.x << "  " << absop.y << "\n";
      }
  }

  for (unsigned int i=0; i<polarCluster.size(); i++) {
    Vec relopvon = Vec(sqrt(polarDistanceBuffer[polarCluster[i].idvon]) * 1000.0, 0) 
      * Angle(dangle * polarCluster[i].idvon );
    Vec absopvon = robot2world*(relopvon);

    Vec relopbis = Vec(sqrt(polarDistanceBuffer[polarCluster[i].idbis]) * 1000.0, 0) 
      * Angle(dangle * polarCluster[i].idbis );
    Vec absopbis = robot2world*(relopbis);

    LOUT << "\% red thin line " 
	     << absopvon.x << "  " << absopvon.y << "  "
	     << absopbis.x << "  " << absopbis.y << "\n";
  }
#endif
}
