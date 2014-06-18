#ifndef _V4LIMAGESOURCE_H_
#define _V4LIMAGESOURCE_H_

#include <string>
#include "ImageSource.h"
#include "../../Structures/TribotsException.h"
#include "../../Fundamental/ConfigReader.h"
#include "Devices/pwc-ioctl.h"

#include <linux/videodev.h>
#include <vector>

#define V4LIMAGESOURCE_DEVICE "/dev/video0"
#define V4LIMAGESOURCE_WIDTH 640
#define V4LIMAGESOURCE_HEIGHT 480

//#define V4LIMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
#define ADJUSTMENT_RATE 3

#ifdef V4LIMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
#include <unicap.h>
#include <unicap_status.h>
#endif

namespace Tribots {

	class V4LImageSource : public ImageSource {
		public:
			V4LImageSource (const ConfigReader&, const std::string&, bool =false) throw (HardwareException, InvalidConfigurationException);
			~V4LImageSource () throw ();
			
			ImageBuffer getImage () throw (ImageFailed);
		
			int getWidth() const throw () {
				return _imgbuf->width;
			};

			int getHeight() const throw () {
				return _imgbuf->height;
			};

      void setCameraParameters(const ConfigReader& cfg, const std::string& section)
          throw (InvalidConfigurationException);

		
		protected:

			/* image acquisition */
			int delay;
			void setupBuffer();
			void setupFormat(int width, int height);
			ImageBuffer *_imgbuf;
			int _frame;
			int _dev;
			int _width;
			int _height;
			std::vector<char*> _buffer;
			std::vector< struct v4l2_buffer > _v4l2_mbuf;

#ifdef V4LIMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
    void setTemperature (unsigned int temperature);
    void setGain (unsigned int gain);
    void setExposure (unsigned int exposure);

    // camera parameters (to be set via Unicap)
    std::vector<int> size, balance_area;
    int balance_rate, framerate;
    unsigned int def_gain, def_temp, def_expo;
    bool soft_balance, soft_exposure, adjust_gain, use_temperature;
    double temp_min, temp_max, gain_min, gain_max, exp_min, exp_max;

    int shutterLogic, backlightCompensation;
    unsigned int frame_count, balanceStep, oldShutter, oldGain;
    int sharpness, contrast;
    double autoExposureError, autoExposureDelta;


    unicap_property_t temperature, exposure, gain;    
    unicap_handle_t handle;
    unicap_device_t device;
    unicap_data_buffer_t *latest;
    unicap_format_t format;
#endif

	};
}


#endif
