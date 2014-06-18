#include "GenericMultiSourceVision.h"

#include <string>
#include <iostream>
#include <sstream>
#include <cmath>

#include "../VisionFactory.h"
//#include "../../WorldModel/WorldModel.h"
#include "../../Structures/Journal.h"
//#include "../../WorldModel/WorldModel.h"
#include "../../Fundamental/geometry.h"

#ifdef REGIONDEBUG
// only for debug purposes
static void debugPrint(const char* msg);
#endif

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::VisionBuilder {
    static Builder the_builder;
  public:
    Builder () {
      Tribots::VisionFactory::get_vision_factory ()->sign_up (std::string("GenericMultiSource"), this);
    }
    Tribots::VisionType* get_vision (const std::string&, const std::string& section, const Tribots::ConfigReader& reader, Tribots::VisionType*) throw (Tribots::TribotsException,std::bad_alloc) {
      return new Tribots::GenericMultiSourceVision (reader, section.c_str());
    }
  };
  Builder the_builder;
}

namespace Tribots {

  using namespace std;

  GenericMultiSourceVision::GenericMultiSourceVision(const ConfigReader& config, const char* section)
    throw (InvalidConfigurationException, HardwareException,
           BadHardwareException, TribotsException)
    : cycles(0), report_computation_time(false), 
      report_computation_time_gnuplot(false), cameraFusionStyle(FUSE_FAILSAFE),multiCameraStats(false)
  {
    vector<string> proc_types;
    vector<string> proc_sections;
    vector<string> prod_sections;

    string configSection = section;
    string str;
    // read configuration

    if (config.get((configSection +"::image_processing_types").c_str(), proc_types) <= 0) {
      throw InvalidConfigurationException("GenericMultiSourceVision::image_processing_types");
    }   
    if (config.get((configSection +"::image_processing_sections").c_str(), proc_sections) <= 0) {
      throw InvalidConfigurationException("GenericMultiSourceVision::image_processing_sections");
    }   
    if (config.get((configSection +"::image_producer_sections").c_str(), prod_sections) <= 0) {
      throw InvalidConfigurationException("GenericMultiSourceVision::image_producer_sections");
    }      
    if (proc_types.size() != proc_sections.size()) {
      throw InvalidConfigurationException("number of image_processing_types does not match number of image_processing_sections");
    }
    if (prod_sections.size() != proc_sections.size()) {
      throw InvalidConfigurationException("number of image_producer_sections does not match number of image_processing_sections");
    }    
    if (config.get("report_computation_time", report_computation_time) < 0) {
      throw InvalidConfigurationException("report_computation_time");
    }  
    if (config.get("report_computation_time_gnuplot", 
		   report_computation_time_gnuplot) < 0) {
      throw InvalidConfigurationException("report_computation_time_gnuplot");
    }
    if (config.get((configSection+"::camera_fusion_style").c_str(),str) >= 0)
    {
      if (str == "failsafe")
        cameraFusionStyle = FUSE_FAILSAFE;
      else if (str == "independent")
        cameraFusionStyle = FUSE_INDEPENDENT;
      else if (str == "fuse")
        cameraFusionStyle = FUSE_3D;
    }
    if (cameraFusionStyle == FUSE_3D)
    {
      // in case of fuse, leave close area 2d (due to cut-region-problems)
      closeNonFuseRange = 1000.; // default: 1m
      config.get((configSection+"::non_fuse_range").c_str(), closeNonFuseRange);
      stringstream ss;
      ss << "closeNonFuseRange: " << closeNonFuseRange << "\n";
      JMESSAGE(ss.str().c_str());
    }
    stringstream ss;
    ss << "camera fusion style: " << cameraFusionStyle << "\n";
    ss << "C FS " << FUSE_FAILSAFE << " C IN " << FUSE_INDEPENDENT << " C 3D " << FUSE_3D << "\n";
    JMESSAGE(ss.str().c_str());

    // multi camera statistics (success vs. misses on robotcontrol console)
    config.get("multi_camera_stats",multiCameraStats);

    // initialize image sources
    
    for (unsigned int i=0; i < prod_sections.size(); i++) {
      imgProds.push_back(new ImageProducer(config, prod_sections[i]));   
    }

    images.resize(imgProds.size());   
    producerTimes.resize(imgProds.size()); 
    imageProcessingTimes.resize(imgProds.size());
    visibleObjects.resize(imgProds.size());
    ballRegionLists.resize(imgProds.size());
    goodFrames.resize(imgProds.size());
    badFrames.resize(imgProds.size());
    
    for (unsigned int i=0; i < imgProds.size(); i++) {
      images[i] = imgProds[i]->nextImage();
      ballRegionLists[i] = 0;
      delete images[i];
    }
        
    // initialize image processings
    
    for (unsigned int i=0; i < proc_types.size(); i++) {
      imgProc.push_back(new ImageProcessing(config, 
                                            proc_types[i].c_str(), 
                                            proc_sections[i].c_str(),
                                            imgProds[i]));   
    }    
  }

  /* lokale Hilfsmethode, die eine Modulnachricht erzeugt. */
  static const char* slip_prepareMsg(string module, int source, double time)
  {
    string str;
    stringstream stream;
    stream << "average " << module << " time for source " << source << " was " 
	   << (time / 1000.) << "ms" << endl;
    getline(stream, str);
    return str.c_str();
  }
  static const char* slip_prepareMsgTotal(string module, double time)
  {
    string str;
    stringstream stream;
    stream << "average " << module << " time for all sources was " 
	   << (time / 1000.) << "ms" << endl;
    getline(stream, str);
    return str.c_str();
  }  

  GenericMultiSourceVision::~GenericMultiSourceVision() throw()
  {  
    Time time;
    float frameRate, effectiveFrameRate;

    if (cycles > 0 && report_computation_time) {
      long productionTotal = 0;
      long processingTotal = 0;
      
      for (unsigned int i=0; i < producerTimes.size(); i++) {
        productionTotal += producerTimes[i];
        processingTotal += imageProcessingTimes[i];
        
        JMESSAGE(slip_prepareMsg("image production", i,
                 static_cast<double>(producerTimes[i]) / cycles));
        JMESSAGE(slip_prepareMsg("image processing", i, 
                 static_cast<double>(imageProcessingTimes[i]) / cycles));
      }
      JMESSAGE(slip_prepareMsgTotal("image production", 
                               static_cast<double>(productionTotal) / cycles));
      JMESSAGE(slip_prepareMsgTotal("image processing", 
                               static_cast<double>(processingTotal) / cycles));
    }      

    // dump statistics for multiple cameras
    for (unsigned int i=0; i < images.size(); i++)
    {
      stringstream ss;
      effectiveFrameRate = ((float)goodFrames[i])/(normalOperationStartingTime.elapsed_msec()/1000.0);
      frameRate = ((float)(badFrames[i]+goodFrames[i]))/(normalOperationStartingTime.elapsed_msec()/1000.0);
      ss.precision(2);
      ss << "statistics for camera " << i << ": fails/overall " << badFrames[i] << ":" << (goodFrames[i]+badFrames[i]) << " = " << ((float)(badFrames[i])/((float)(goodFrames[i]+badFrames[i]))*100.0) << "%";
      ss << " (frameRate: " << frameRate << " / effective framerate: " << effectiveFrameRate << ")";
      JMESSAGE(ss.str().c_str());
    }

    for (unsigned int i=0; i < imgProc.size(); i++) {
      delete imgProc[i];
      delete imgProds[i];
    }
  }

  int
  GenericMultiSourceVision::get_num_sources() const throw()
  { return imgProds.size(); } 

  void 
  GenericMultiSourceVision::process_images() 
    throw (Tribots::BadHardwareException)
  {
    cycles++;
    if (cycles == 10) {
      normalOperationStartingTime.update();
    }
    Time time; 
    
    //
    // when two or more unsynchronized cameras are used,
    // small differences in the framerate can lead to capture problems
    // that occur for e.g. 5 seconds followed by e.g. 10 seconds of correct
    // functionality.
    // in order to combat this situation, we first 
    // - foreach camera: take a shot at grabbing an image from all sources
    // - foreach camera: for the failed grabbings, grab again, then process
    //
    // this leads to the desired behaviour that the second camera is,
    // in the failure case, grabbed as follows:
    // [-- grab camera 0 --][--grab camera 1--][--process camera 0--][--grab camera 1--][--process camera 1--]
    //                      [----------- optional delay  -----------]
    //
    // this optional delay allows to grab an image, even if the two cameras are unsynchronized
    //
    
    for (unsigned int i=0; i < images.size(); i++)
    {
      try // catch ImageFailed (for each camera)
      {
        time.update();
        images[i] = 0;
        images[i] = imgProds[i]->nextImage();
        producerTimes[i] += time.elapsed_usec();
        goodFrames[i]++;
      } catch (ImageFailed&)
      {
        if (images[i])
        {
            delete images[i];
            images[i]=0;
        }
        // do not perform statistics here, but in the second try (in case it failed here)
      } 
    } // foreach source 
    
    for (unsigned int i=0; i < images.size(); i++) // image processing
    {
      time.update();
      visibleObjects[i].objectlist.clear();              // die alten Objekte loeschen
      if (ballRegionLists[i])                            // die alten RegionLists loeschen
      {
        delete ballRegionLists[i];
        ballRegionLists[i] = 0;
      }
      ballRegionLists[i] = new RegionList();
      
      if (!images[i]) // if capture has failed in the last try, try again now
      {
        try
        {
          images[i] = imgProds[i]->nextImage();
          producerTimes[i] += time.elapsed_usec();
          goodFrames[i]++;
        } catch (ImageFailed&)
        {
          badFrames[i]++;
          if (images[i])
          {
              delete images[i];
              images[i]=0;
          }
          stringstream ss;
          ss << "failed in grabbing an image from source " << i;
          JERROR (ss.str().c_str());
        //  LOUT << "image-grab (retry) failed for source " << i << "\n";
        }
      }
      
      if (images[i]) // nur wenn ein Bild vorhanden ist
      {
        imgProc[i]->process_image(images[i], &visibleObjects[i], ballRegionLists[i]);
        imageProcessingTimes[i] += time.elapsed_usec();
        visibleObjects[i].timestamp = images[i]->getTimestamp();
      }
    }

#ifdef REGIONDEBUG    
    debugPrint("                                                          ");
#endif
    
    if (images.size() == 1)
    {
      // only one camera, no need to fuse anything
      if (images[0] && ballRegionLists[0]->list.size())
      {
        // image was processed, ball was found
        Line3D ballLine = imgProds[0]->getImageWorldMapping()->map3D(ballRegionLists[0]->list[0]->getCenterOfGravity());
        Vec3D ballPos = ballLine.intersectZPlane(110.); // get the ball on the ground
        Vec   ballPos2D = imgProds[0]->getImageWorldMapping()->map(ballRegionLists[0]->list[0]->getCenterOfGravity());
        // only give back ball if its within the field
//	       LOUT << "visibleObjcts[0].timestamp=" << visibleObjects[0].timestamp <<endl;
//        const RobotLocation& rloc = MWM.get_robot_location(visibleObjects[0].timestamp);
//        const FieldGeometry& fg = MWM.get_field_geometry ();
        double maxY = 15000;//0.5*fg.field_length+fg.goal_band_width;
        double maxX = 20000;//0.5*fg.field_width+fg.side_band_width;
        //Frame2d rel2abs(rloc.pos, rloc.heading);
        Frame2d rel2abs(Vec(0,0), Angle(0));
        Vec firstWorld = rel2abs * ballPos.toVec();
        if (ballPos2D.length() < 100000 && // closer than 100m
            fabs(firstWorld.x) < maxX &&
            fabs(firstWorld.y) < maxY) {
          visibleObjects[0].objectlist.push_back(VisibleObject (ballPos.toVec(), VisibleObject::ball));
//	        LOUT << "Ein Bild, " << ballRegionLists[0]->list.size() << " bekommen, folgenden ins WM gegeben: " << ballPos << endl;
	      }
	      else {
//          LOUT << "Ein Bild, " << ballRegionLists[0]->list.size() << " bekommen, folgenden NICHT ins WM gegeben: " << ballPos << endl;
             }
      }
    } else if (images.size() >= 2)
    {
      // multi cameras, fusion depends on cameraFusionStyle
//      const RobotLocation& rloc = MWM.get_robot_location(visibleObjects[0].timestamp);
//      const FieldGeometry& fg = MWM.get_field_geometry ();
      //double maxY = 0.5*fg.field_length+fg.goal_band_width;
      //double maxX = 0.5*fg.field_width+fg.side_band_width;
      double maxY = 15000;
      double maxX = 20000;
      Frame2d rel2abs(Vec(0,0),Angle(0));
      Vec OmniBall, DirBall; // 2D w/o intersect-theorem
      Line3D OmniLine, DirLine; // 3D Lines through ball and camera origins
      Vec3D OmniBall3D, DirBall3D; // individual positions w/ intersect-theorem, but not fused
      bool foundOmniBall = false, foundDirBall = false;
      bool inFieldOmni = false, inFieldDir = false;
      
//      LOUT << "ballRegionLists[0].size " << ballRegionLists[0]->list.size() << " ballRegionLists[1]->list.size() " << ballRegionLists[1]->list.size() << "\n";
      
      // calculate ball positions
      if (ballRegionLists[0]->list.size())
      {
        // found a ball in the first camera
        OmniBall   = imgProds[0]->getImageWorldMapping()->map  (ballRegionLists[0]->list[0]->getCenterOfGravity());
        OmniLine   = imgProds[0]->getImageWorldMapping()->map3D(ballRegionLists[0]->list[0]->getCenterOfGravity());
        OmniBall3D = OmniLine.intersectZPlane(110.); // get the ball on the ground
        foundOmniBall = true;
        Vec firstWorld = rel2abs * OmniBall3D.toVec();
        if (OmniBall.length() < 100000 && // closer than 100m
            fabs(firstWorld.x) < maxX &&
            fabs(firstWorld.y) < maxY)
          inFieldOmni = true;
      }
      if (ballRegionLists[1]->list.size())
      {
        // found a ball in the second camera
        DirBall   = imgProds[1]->getImageWorldMapping()->map  (ballRegionLists[1]->list[0]->getCenterOfGravity());
        DirLine   = imgProds[1]->getImageWorldMapping()->map3D(ballRegionLists[1]->list[0]->getCenterOfGravity());
        DirBall3D = DirLine.intersectZPlane(110.); // get the ball on the ground
        foundDirBall = true;
        Vec firstWorld = rel2abs * DirBall3D.toVec();
        if (DirBall.length() < 100000 && // closer than 100m
            fabs(firstWorld.x) < maxX &&
            fabs(firstWorld.y) < maxY)
          inFieldDir = true;
      }
      
      // FAILSAFE: only use information from additional cameras if primary camera does not yield (ball) information
      if (cameraFusionStyle == FUSE_FAILSAFE)
      {
        // wenn Ball in Omnikamera im Feld, dann nimm den.
        if (inFieldOmni)
        {
//          LOUT << "FUSE_FAILSAFE: found omni in field\n";
          visibleObjects[0].objectlist.push_back(VisibleObject (OmniBall3D.toVec(), VisibleObject::ball));
        }
        else if (inFieldDir) // wenn Ball in Gerichteter im Feld, dann nimm den
        {
//          LOUT << "FUSE_FAILSAFE: found directional in field\n";
          visibleObjects[1].objectlist.push_back(VisibleObject (DirBall3D.toVec(), VisibleObject::ball));
        } else
        {
//          LOUT << "FUSE_FAILSAFE: no balls found in field\n";
        }
         /*
        else if (foundOmniBall) // sonst, Ball aus Omnikamera falls moeglich
        {
          visibleObjects[0].objectlist.push_back(VisibleObject (OmniBall3D.toVec(), VisibleObject::ball));          
        }*/
      }
      // INDEPENDENT: use information from all cameras (indepedently) but do not fuse
      else if (cameraFusionStyle == FUSE_INDEPENDENT)
      {
        if (inFieldOmni) // wenn Ball in Omnikamera im Feld, dann nimm den.
        {
//          LOUT << "FUSE_INDEPENDENT: found omni in field\n";
          visibleObjects[0].objectlist.push_back(VisibleObject (OmniBall3D.toVec(), VisibleObject::ball));
          if ((foundDirBall) &&                     // gerichteten Ball gesehen?
              (DirBall.length() < 100000) &&        // gerichteter Ball ist unterhalb des Horizonts?
              ((OmniBall-DirBall).length() < 1000.) // wenn gerichteter Ball an hnlicher Stelle
             )
          {
//            LOUT << "FUSE_INDEPENDENT: also found directional in field\n";
            visibleObjects[1].objectlist.push_back(VisibleObject (DirBall3D.toVec(), VisibleObject::ball));
          }
        }
        else if (inFieldDir) // Wenn Ball aus Omni nicht im Feld und Ball in gerichteter im Feld
        {
//          LOUT << "FUSE_INDEPENDENT: found directional in field\n";
          visibleObjects[1].objectlist.push_back(VisibleObject (DirBall3D.toVec(), VisibleObject::ball));
        } else
        {
//          LOUT << "FUSE_INDEPENDENT: no balls found in field\n";
        }
        /*
        else if (foundOmniBall) // sonst, Ball aus Omnikamera falls moeglich
        {
          visibleObjects[0].objectlist.push_back(VisibleObject (OmniBall3D.toVec(), VisibleObject::ball));          
        } */
      }
      // FUSE_3D: if possible, fuse camera information yielding stereoscopic depth
      else if (cameraFusionStyle == FUSE_3D)
      {
        if (foundOmniBall && !foundDirBall && inFieldOmni) // ball *nur* in der omnikamera
        {
          visibleObjects[0].objectlist.push_back(VisibleObject (OmniBall3D.toVec(), VisibleObject::ball));
#ifdef REGIONDEBUG
          debugPrint("FUSE: omni only");
#endif
//          LOUT << "FUSE_3D: found omni ball only\n";
        }
        else if (!foundOmniBall && foundDirBall && inFieldDir) // ball *nur* in der gerichteten kamera
        {
          visibleObjects[1].objectlist.push_back(VisibleObject (DirBall3D.toVec(), VisibleObject::ball));
#ifdef REGIONDEBUG
          debugPrint("FUSE: dir only");
#endif
//          LOUT << "FUSE_3D: directional ball only\n";
        }
        else if (foundOmniBall && foundDirBall) // ball in beiden
        {
          Vec3D FusedBall3D = intersectSkewLines(OmniLine,DirLine);
          FusedBall3D.z -= 110.; // z=0 wenn der ball auf dem boden ist
//          LOUT << "fusedBall3D.z == " << FusedBall3D.z << " OmniBall3D.length() == " << OmniBall3D.length() << " (closeNonFuseRange: " << closeNonFuseRange << ")\n";
          if (
              (fabs((OmniLine.p2-OmniLine.p1).toVec().angle((DirLine.p2-DirLine.p1).toVec()).get_deg_180()) <= 3.0)
              &&
              (OmniBall3D.length() > closeNonFuseRange) // do not fuse in close range
              &&
              (FusedBall3D.z >= -50.) // only balls above ground (+ noise)
             )
          {
            // intersect the two ball lines
            visibleObjects[0].objectlist.push_back(VisibleObject (FusedBall3D.toVec(), VisibleObject::ball3d, 0, FusedBall3D.z));
#ifdef REGIONDEBUG
            debugPrint("FUSE: 3d ball");
#endif
//            LOUT << "FUSE_3D: found 3d ball which is not too close and above ground\n";
          }
          else if (inFieldOmni) // winkel zu gross, 2d aus der omni nehmen
          {
            visibleObjects[0].objectlist.push_back(VisibleObject (OmniBall3D.toVec(), VisibleObject::ball));
#ifdef REGIONDEBUG
            debugPrint("FUSE: omni, not fusing");
#endif
//            LOUT << "FUSE_3D: not fusing, so giving 2d ball omni in field\n";
          } else
          {
//            LOUT << "FUSE_3D: no omni ball in field\n";
          }
        } // ball in beiden
        else {
//          LOUT << "FUSE_3D: no balls in fields. OmniBall3D " << OmniBall3D << " DirBall3D " << DirBall3D << "\n";
        }
      } // fUSE_3D
    } // images.size() >= 2
    
    // give visual object information to world model and destroy images
    for (unsigned int i=0; i < images.size(); i++)
    {
      if (images[i])
      {
        //MWM.set_visual_information(visibleObjects[i], i);
        delete images[i];
        images[i]=0;
      }
    }

    // statistics
    if (multiCameraStats)
    {
      Time time;
      float frameRate, effectiveFrameRate;
      
      for (unsigned int i=0; i < images.size(); i++)
      {
        effectiveFrameRate = ((float)goodFrames[i])/(time.get_msec()/1000.0);
        frameRate = ((float)(badFrames[i]+goodFrames[i]))/(time.get_msec()/1000.0);
      
        std::stringstream inout;
        std::string line;
        inout << " MultiCameraStats: Fails/Overall " << badFrames[i] << ":" << (goodFrames[i]+badFrames[i]) << " = " << ((float)(badFrames[i])/((float)(goodFrames[i]+badFrames[i]))*100.0) << "%";
        inout << " FR: " << frameRate << " EFR: " << effectiveFrameRate << "                                                 ";
        std::getline (inout, line);
      }
    }
  }  // end process_images

  const vector<RegionList*>& GenericMultiSourceVision::get_regionlists() throw(TribotsException)
  {
    return ballRegionLists;
  }

  void GenericMultiSourceVision::request_image_raw(int source) throw(TribotsException)
  {
    return imgProc[source]->request_image_raw();
  }
      
  void GenericMultiSourceVision::request_image_processed(int source) throw(TribotsException)
  {
    imgProc[source]->request_image_processed();
  }

  bool GenericMultiSourceVision::is_image_available(int source) throw()
  {
    return imgProc[source]->is_image_available();
  }
  
  const Image* GenericMultiSourceVision::get_image(int source) 
    throw(TribotsException)
  {
    return imgProc[source]->get_image();
  }

  void GenericMultiSourceVision::free_image(int source) throw(TribotsException)
  {
    imgProc[source]->free_image();
  }

}

#ifdef REGIONDEBUG
// debug
static void debugPrint(const char* msg)
{
        std::stringstream inout;
        std::string line;
        move (23,0);
        inout << msg;
        std::getline (inout, line);
        addstr(line.c_str());
}
#endif
