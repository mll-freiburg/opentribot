#ifndef _PS3IMAGESOURCE_H_
#define _PS3IMAGESOURCE_H_

#include <string>
#include "ImageSource.h"
#include "../../Structures/TribotsException.h"
#include "../../Fundamental/ConfigReader.h"
#include "V4L_SetControls.h"

//#include <linux/videodev.h>

#include <linux/videodev2.h>

#include <vector>

#define PS3IMAGESOURCE_DEVICE "/dev/video1"
#define PS3IMAGESOURCE_WIDTH 640
#define PS3IMAGESOURCE_HEIGHT 480

//#define PS3IMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
#define ADJUSTMENT_RATE 3

#ifdef PS3IMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
#include <unicap.h>
#include <unicap_status.h>
#endif

namespace Tribots {

	class PS3ImageSource : public ImageSource {
		public:
			PS3ImageSource (const ConfigReader&, const std::string&, bool =false) throw (HardwareException, InvalidConfigurationException);
			~PS3ImageSource () throw ();
			
			ImageBuffer getImage () throw (ImageFailed);
		
			int getWidth() const throw () {
				return _imgbuf->width;
			};

			int getHeight() const throw () {
				return _imgbuf->height;
			};

      void setCameraParameters()
          throw (InvalidConfigurationException);


int imageWidth;
int imageHeight;

int red_balance;
int blue_balance;
int exposure;
int autogain;
int main_gain;
int hflip;
int vflip;
int sharpness;
double autoExposureDelta;
double autoExposureError;
int shutterMax;
int shutterMin;
		
int framecount;
		protected:

         V4l2_Controls controls;

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
		


  int bAX;                               ///< Balance are    a Position
   int bAY;                               ///< Balance are    a Position
   int bAWidth;                           ///< Balance are    a Breite
   int bAHeight;                          ///< Balance are    a Hoehe

 
 
    void setBalanceArea(int x, int y, int w, int h);
    void getBalanceArea(int *x, int *y, int *w, int *h);


void doSoftwareExposure();
void doSoftwareGain();

   void doSoftwareBalance();







#ifdef PS3IMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
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
