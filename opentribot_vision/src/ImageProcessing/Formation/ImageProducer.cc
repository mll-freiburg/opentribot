#include "ImageProducer.h"

#include <string>

#include "ImageSourceFactory.h"
#include "RGBImage.h"
#include "YUVImage.h"
#include "UYVYImage.h"
#include "FileMonitor.h"
//#include "SharedMemWriter.h"
#include "JPEGIO.h"
#include "PPMIO.h"
#include "BufferedIO.h"
#include "../PixelAnalysis/YUVLookupTable.h"
#include "../PixelAnalysis/Image2WorldMapping.h"
#include "../PixelAnalysis/OmniCameraMapping.h"
#include "../PixelAnalysis/DirectionalCameraMapping.h"
#include "../../Fundamental/stringconvert.h"

using namespace Tribots;
using namespace std;

namespace {
  
  typedef struct {
    string key;
    int value;
  } KeyValueEntry;

  static const int image_buffer_types_size = 3;
  static KeyValueEntry image_buffer_types[image_buffer_types_size] = 
    { { "YUV",  ImageProducer::TYPE_YUVImage  },
      { "RGB",  ImageProducer::TYPE_RGBImage  },
      { "UYVY", ImageProducer::TYPE_UYVYImage } };

}    

    
ImageProducer::ImageProducer(const ConfigReader& config, bool force_blocking) throw(TribotsException)
  : imgSource(0), classifier(0), monitor(0), 
    imageType(TYPE_YUVImage), imageCenter (-1,-1), 
    image2world (NULL), world2image(NULL), robotMask(NULL), camera_delay (0)
{
  std::vector<std::string> src_desc;
  if (config.get("image_sources", src_desc)<=0)
    throw InvalidConfigurationException ("image_sources");
  ImageProducer::init(config, src_desc[0], force_blocking);
}

ImageProducer::ImageProducer(const ConfigReader& config, 
                               const std::string& image_handler, bool force_blocking)
      throw(TribotsException)
  : imgSource(0), classifier(0), monitor(0), 
  imageType(TYPE_YUVImage), imageCenter (-1,-1), 
  image2world (NULL), world2image(NULL), robotMask(NULL), camera_delay (0)
{
  ImageProducer::init(config, image_handler, force_blocking);
}

    
void ImageProducer::init(const ConfigReader& config, 
                           const string& image_handler, bool force_blocking) 
{
  string imgSourceType;
  string image_source;
  
  initHandler (image_source, imgSourceType, image_handler, config);
  ConfigReader config2 (config);
  if (force_blocking)
    config2.set ((image_source+"::blocking").c_str(), 1);
  imgSource = ImageSourceFactory::get_image_source_factory()->get_image_source (imgSourceType, config2, image_source);
  initColorClassifier (image_source, config);
  initMonitor (image_handler, config);
  initMappings (image_source, config);
  initRobotMask (image_source, config);
}

void ImageProducer::initHandler (std::string& imageSourceSectionName, std::string& sourceType, const std::string& imageHandlerSectionName, const ConfigReader& config) {
  string sTmp;
        
  if (config.get((imageHandlerSectionName+"::image_source").c_str(), imageSourceSectionName)<=0)
    throw InvalidConfigurationException((imageHandlerSectionName+"::image_source").c_str());
  if(config.get((imageHandlerSectionName+"::analysis_format").c_str(), sTmp)<=0)
    throw InvalidConfigurationException((imageHandlerSectionName+"::analysis_format").c_str());
  for (int i=0; i < image_buffer_types_size; i++) {
    if (image_buffer_types[i].key == sTmp) {
      imageType = image_buffer_types[i].value;
    }
  }    
  if (config.get((imageSourceSectionName+"::image_source_type").c_str(), 
      sourceType) <= 0) {
        throw InvalidConfigurationException((
            imageSourceSectionName+"::image_source_type").c_str());
      }
}
  
void ImageProducer::initColorClassifier (const std::string& imageSourceSectionName, const ConfigReader& config) {
  std::string classifierType, sTmp;
  int ret;
  ret = config.get((imageSourceSectionName+"::color_classifier_type").c_str(), classifierType);
  if (ret < 0)
    throw InvalidConfigurationException((imageSourceSectionName+"::color_classifier_type").c_str());
  else if (ret > 0) {
    if (classifierType == "YUVLut")
      classifier = new YUVLookupTable();
    else 
      throw TribotsException ((string("classifierType ")+classifierType+" not yet implemented.").c_str());
      
    ret = config.get((imageSourceSectionName+"::color_classifier_file").c_str(), sTmp);
    if (ret < 0)
      throw InvalidConfigurationException((imageSourceSectionName+"::color_classifier_file").c_str());
    else if (ret > 0) {
      classifier->load(sTmp);
//      classifier->save(("test"+image_handler+".lut").c_str());
    }
  }  
}

void ImageProducer::initMonitor (const std::string& imageHandlerSectionName, const ConfigReader& config) {
  string monitorSection;
  int ret;
  string filenameBase;
  bool singleFile = false;
  int step=1;
  int buffers=100;
  string sTmp = "PPM";
  int quality=50;
  if ((ret=config.get((imageHandlerSectionName+"::monitor").c_str(), monitorSection)) < 0)
    throw InvalidConfigurationException((imageHandlerSectionName+"::monitor").c_str());
  else if (ret > 0) {
    if (config.get((monitorSection+"::filename_base").c_str(), filenameBase) <= 0)
      throw InvalidConfigurationException((monitorSection+"::monitor_filename_base").c_str());
    if (config.get((monitorSection+"::single_file").c_str(), singleFile) < 0)
      throw InvalidConfigurationException((monitorSection+"::single_file").c_str());
    if (config.get((monitorSection+"::step").c_str(), step) <= 0)
      throw InvalidConfigurationException((monitorSection+"::step").c_str());
    if (config.get((monitorSection+"::file_type").c_str(), sTmp) < 0)
      throw InvalidConfigurationException((monitorSection+"::file_type").c_str());
    if (sTmp=="PPM") {  // hierfuer eventuell auch eine factory
      monitor = new FileMonitor(new PPMIO(), filenameBase, step, singleFile);
    } else if (sTmp=="BufferedPPM") {
      if (config.get("BufferedIO::buffers", buffers) <= 0)
        throw InvalidConfigurationException("BufferedIO::buffers");
      monitor = new FileMonitor(new BufferedIO(buffers, new PPMIO()), filenameBase, step, singleFile);
    } else if (sTmp=="BufferedJPEG") {
      if (config.get("BufferedIO::buffers", buffers) <= 0)
        throw InvalidConfigurationException("BufferedIO::buffers");
      if (config.get("JPEGIO::quality", quality) <= 0)
        throw InvalidConfigurationException("JPEGIO::quality");
      monitor = new FileMonitor(new BufferedIO(buffers, new JPEGIO(quality)), filenameBase, step, singleFile);
    } else if (sTmp=="JPEG") {
      if (config.get("JPEGIO::quality", quality) <= 0)
        throw InvalidConfigurationException("JPEGIO::quality");
      monitor = new FileMonitor(new JPEGIO(quality), filenameBase, step, singleFile);
    } else if (sTmp=="SHM") {
      //monitor = new SharedMemWriter(new JPEGIO(quality),filenameBase,step,singleFile);
    } else
      throw InvalidConfigurationException((monitorSection+"::file_type").c_str());
  }
}
  
void ImageProducer::initMappings (const std::string& imageSourceSectionName, const ConfigReader& config) {
  string mappingFile = "";
  string mappingType = "";
  vector<double> vd;
  imageCenter = Vec(-1,-1);
  if (config.get((imageSourceSectionName+"::mapping_type").c_str(), mappingType) <= 0) {
    throw InvalidConfigurationException((imageSourceSectionName+"::mapping_type").c_str());
  }
  if (config.get((imageSourceSectionName+"::mapping_file").c_str(), mappingFile) <= 0) {
    throw InvalidConfigurationException((imageSourceSectionName+"::mapping_file").c_str());
  }
  if (config.get((imageSourceSectionName+"::image_center").c_str(), vd) <0) {
    throw InvalidConfigurationException((imageSourceSectionName+"::image_center").c_str());
  }
  if (vd.size()>=2)
    imageCenter = Vec (vd[0], vd[1]);
    
  // fuer jeden kamera typ beim ImageProducer zu erledigen:
  // a) die beien mappings fuellen, b) imageCenter setzen, c) Origin setzen
  if (mappingType == "omni") { // Omnikamera braucht: ImageCenter und Origin aus ConfigFile
    double height;
    if (config.get((imageSourceSectionName+"::camera_height").c_str(), height) <= 0) {
      throw InvalidConfigurationException((imageSourceSectionName+"::camera_height").c_str());
    }
    cameraOrigin = Vec3D(0,0,height);
    ImageWorldOmniMapping iw_marker (mappingFile, cameraOrigin, 
                                     (int)imageCenter.x, 
                                     (int)imageCenter.y);
    
    world2image = new WorldImageOmniMapping (mappingFile, cameraOrigin,
                                             (int)imageCenter.x,
                                             (int)imageCenter.y);
    if (imageCenter.x<=0 || imageCenter.y<=0) {
      imageCenter = iw_marker.getImageCenter();
    }
    image2world = new ImageWorldLUTMapping(iw_marker);
  }
  else if (mappingType == "directed") { // Directedkamera braucht: format7-offset
    int xOffset = imgSource->getOffsetX();
    int yOffset = imgSource->getOffsetY();

    ImageWorldDirectionalMapping iw_directed(mappingFile,getWidth(),getHeight(),xOffset,yOffset);
    world2image = new WorldImageDirectionalMapping(mappingFile,getWidth(),getHeight(),xOffset,yOffset);
    image2world = new ImageWorldLUTMapping (iw_directed);

    cameraOrigin = iw_directed.getOrigin();
    if (imageCenter.x <= 0 || imageCenter.y <= 0) {
      imageCenter = Vec(getWidth()/2, getHeight()/2); // mal in die Mitte vom Bild
    }
  }
  else {
    throw InvalidConfigurationException((imageSourceSectionName+"::origin").c_str());
  }
}

void ImageProducer::initRobotMask (const std::string& imageSourceSectionName, const ConfigReader& config) {
  string robotMaskFile ="";
  config.get((imageSourceSectionName+"::robot_mask_file").c_str(), robotMaskFile);
  try{
    if (robotMaskFile.length()>0)
      robotMask = new RobotMask (robotMaskFile.c_str());
  }catch(TribotsException& e){
    cerr << e.what() << '\n';
    robotMask=NULL;
  }
}
  
  
  
ImageProducer::~ImageProducer()
{
  if (imgSource)
    delete imgSource;
  if (classifier)
    delete classifier;
  if (monitor)
    delete monitor;
  if (image2world)
    delete image2world;
  if (world2image)
    delete world2image;
  if (robotMask)
    delete robotMask;
}


    
Image* ImageProducer::nextImage() throw (ImageFailed, BadHardwareException)
{
  ImageBuffer buffer;
  buffer = imgSource->getImage();
    
  Image* image;

  try {
    switch (imageType) {
      case TYPE_RGBImage:  image = new RGBImage (buffer); break;
      case TYPE_UYVYImage: image = new UYVYImage(buffer); break;
      default:             image = new YUVImage (buffer); break;
    }
  } catch (TribotsException& e) {
      // TODO: Journal Ausgabe
    string msg =
        __FILE__
        ": There has occured an error while creating an Image arround "
        "the ImageBuffer that has been received from the ImageSource. "
        "It is most likely, that there is no implementation of the "
        "needed conversion method to convert the format of the ImageBuffer "
        " to the format used by the selected Image class.\n\n"
        "Solution: Edit the configuration files and select another Image "
        "class that is compatible to the selected ImageSource.";
    msg += string("\n\nOriginal Exception: ") + e.what();
    throw BadHardwareException(msg.c_str());
  }
  if (classifier != 0) {
    image->setClassifier(classifier);
  }
 // cout << "Sizeofbuffer"<<image->getImageBuffer().size<<endl;
  if (monitor) {
    monitor->monitor(image->getImageBuffer(), image->getTimestamp(), image->getTimestamp());
  }

  return image;
}

ImageSource* ImageProducer::getImageSource () throw () {
  return imgSource;
}
  
Vec ImageProducer::getImageCenter () const throw () {
  return imageCenter;
}
  
const ImageWorldMapping* ImageProducer::getImageWorldMapping () const throw () {
  return image2world;
}
  
const WorldImageMapping* ImageProducer::getWorldImageMapping () const throw () {
  return world2image;
}
  
const RobotMask* ImageProducer::getRobotMask () const throw () {
  return robotMask;
}

void ImageProducer::setRobotMask (const RobotMask& newmask) throw () {
  if (robotMask)
    (*robotMask) = newmask;
  else
    robotMask = new RobotMask (newmask);
}
