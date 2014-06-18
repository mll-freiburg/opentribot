
#include "MultiImageProducer.h"

namespace Tribots {

  using std::string;

  MultiImageProducer::MultiImageProducer(const ConfigReader& config)
  {
    // get values from MultiImageProducer section  //////////////////////////

    std::vector<std::string> src_descriptors;
    if (config.get ("image_sources", src_descriptors)<=0)
      throw InvalidConfigurationException("image_sources");
    
      // initialize all image producers
    for(unsigned int i=0; i<src_descriptors.size(); i++)
      producers.push_back(new ImageProducer(config, src_descriptors[i]));

  }

  MultiImageProducer::~MultiImageProducer()
  {
     for(unsigned int i=0; i<producers.size(); i++)
        delete producers[i];
  }


  Image*
  MultiImageProducer::nextImage() throw (ImageFailed, BadHardwareException)
  {
     return nextImage(0);
  }


  Image*
  MultiImageProducer::nextImage(int cam_id) throw (ImageFailed, BadHardwareException)
  {
	if((int)producers.size() > cam_id)
	   return (producers[cam_id])->nextImage();
	else
	   throw BadHardwareException("MultiImageProducer::nextImage(int) : Tried to aquire an image from an unknown camera id.");

  }
  
  
  Vec MultiImageProducer::getImageCenter (unsigned int producer_id) const throw () {
    return (producer_id<producers.size() ? producers[producer_id]->getImageCenter () : Vec(-1,-1));
  }
    
  const ImageWorldMapping* MultiImageProducer::getImageWorldMapping (unsigned int producer_id) const throw ()  {
    return (producer_id<producers.size() ? producers[producer_id]->getImageWorldMapping () : NULL);
  }
  
  const WorldImageMapping* MultiImageProducer::getWorldImageMapping (unsigned int producer_id) const throw ()  {
    return (producer_id<producers.size() ? producers[producer_id]->getWorldImageMapping () : NULL);
  }
  
  const RobotMask* MultiImageProducer::getRobotMask (unsigned int producer_id) const throw () {
    return (producer_id<producers.size() ? producers[producer_id]->getRobotMask () : NULL);
  }

  
  
} /* namespace Tribots */

