#include "V4LImageSource.h"
#include "ImageSourceFactory.h"
#include "Devices/VideoUtils.h"
#include <cmath>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>
#include <stdlib.h>

#include "../../Fundamental/Time.h"

#include <cstring>

#define EOUT( __x__ ) std::cerr << "      #Error: [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "      \n"
#define IOUT( __x__ ) std::cerr << "      #INFO : [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "      \n"

using namespace std;
using namespace Tribots;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace {
  class Builder : public Tribots::ImageSourceBuilder {
    public:
      Builder () {
        Tribots::ImageSourceFactory::get_image_source_factory ()->sign_up (string("V4L"), this);
      }
      Tribots::ImageSource* get_image_source (const std::string&, const Tribots::ConfigReader& reader, const std::string& section) throw (Tribots::TribotsException,std::bad_alloc) {
        return new Tribots::V4LImageSource (reader, section);
      }
  };
  Builder the_builder;
}


namespace Tribots {

V4LImageSource::V4LImageSource (const ConfigReader& cfg, const std::string& section, bool) throw (HardwareException, InvalidConfigurationException)
{
  setCameraParameters(cfg, section);

	/* initialize V4L device */

	string device;
	if (cfg.get((section+"::device_name").c_str(), device) <= 0)
		throw InvalidConfigurationException((section+"::device_name").c_str());

	_dev = open( V4LIMAGESOURCE_DEVICE, O_RDWR, 0 );
        if (-1 == _dev)
		throw HardwareException("V4L: Failed to open device.");

        struct v4l2_capability v4l2cap;
        if (ioctl(_dev, VIDIOC_QUERYCAP, &v4l2cap) == -1)
                throw HardwareException("V4L: Not a V4L2 device");

        if (false == (V4L2_CAP_VIDEO_CAPTURE & v4l2cap.capabilities))
                throw HardwareException("V4L: Does not support VIDEO_CAPTURE");
	
	/* prepare buffers */
	
	setupBuffer();
	setupFormat(V4LIMAGESOURCE_WIDTH, V4LIMAGESOURCE_HEIGHT);
	_frame = 0;

	if (cfg.get((section+"::delay").c_str(), delay) <= 0)
        	throw InvalidConfigurationException((section+"::delay").c_str());
}

V4LImageSource::~V4LImageSource() throw ()
{
	// TODO anything to close? throws errors at quit
}

ImageBuffer V4LImageSource::getImage() throw (ImageFailed)
{
	struct v4l2_buffer* vb = &_v4l2_mbuf[ _frame ];

	if( 0 == ioctl( _dev, VIDIOC_DQBUF, vb ) )
	{
		// cout << "using buffer " << _frame << endl;

		_imgbuf->timestamp.update();
		_imgbuf->timestamp.add_msec(delay);
		_imgbuf->size = vb->bytesused;
		_imgbuf->buffer = (unsigned char*)realloc(
			(void *)_imgbuf->buffer, _imgbuf->size);

		memcpy(_imgbuf->buffer, _buffer[_frame], vb->bytesused); 
		int length = vb->bytesused;
		uint16_t *words = (uint16_t*) _imgbuf->buffer;
		while (length>=2) {
			*words = ((*words >> 8) & 0xff) | ((*words & 0xff) << 8);
			words++;
			length -=2;
		}
	}
	else
	{
		cerr << "IOCTL VIDIOC_DQBUF: " << strerror(errno) << endl;
        	return NULL;
	}

	if ( ioctl( _dev, VIDIOC_QBUF, vb ) == -1 )
	{
		cerr << "IOCTL VIDIOC_QBUF" << endl;
        	return NULL;
	}

	_frame = ( _frame + 1 ) % _v4l2_mbuf.size();
	return *_imgbuf; 
}


void V4LImageSource::setCameraParameters(const ConfigReader& cfg, const std::string& section)
    throw (InvalidConfigurationException)
{
#ifdef V4LIMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
    int dev_nr;
    
    if (cfg.get((section+"::image_size").c_str(), size) <= 0)
        throw InvalidConfigurationException((section+"::image_size").c_str());
    if (cfg.get((section+"::video_device_number").c_str(), dev_nr) <= 0)
        throw InvalidConfigurationException((section+"::video_device_number").c_str());

    if (cfg.get((section+"::delay").c_str(), delay) <= 0)
        throw InvalidConfigurationException((section+"::delay").c_str());
    if (cfg.get((section+"::white_balance_area").c_str(), balance_area) <= 0)
        throw InvalidConfigurationException((section+"::white_balance_area").c_str());
    if (cfg.get((section+"::soft_exposure").c_str(), soft_exposure) <= 0)
        throw InvalidConfigurationException((section+"::soft_exposure").c_str());
    if (cfg.get((section+"::soft_white_balance").c_str(), soft_balance) <= 0)
        throw InvalidConfigurationException((section+"::soft_white_balance").c_str());
    if (cfg.get((section+"::temperature_only").c_str(), use_temperature) <= 0)
        throw InvalidConfigurationException((section+"::temperature_only").c_str());
    if (cfg.get((section+"::white_balance_rate").c_str(), balance_rate) <= 0)
        throw InvalidConfigurationException((section+"::white_balance_rate").c_str());
    if (cfg.get((section+"::framerate").c_str(), framerate) <= 0)
        throw InvalidConfigurationException((section+"::framerate").c_str());
    if (cfg.get((section+"::adjust_gain").c_str(), adjust_gain) <= 0)
        throw InvalidConfigurationException((section+"::adjust_gain").c_str());
    if (cfg.get((section+"::default_gain").c_str(), def_gain) <= 0)
        throw InvalidConfigurationException((section+"::default_gain").c_str());
    if (cfg.get((section+"::default_exposure").c_str(), def_expo) <= 0)
        throw InvalidConfigurationException((section+"::default_exposure").c_str());
    if (cfg.get((section+"::default_temperature").c_str(), def_temp) <= 0)
        throw InvalidConfigurationException((section+"::default_temperature").c_str());
    if (cfg.get((section+"::balance_step").c_str(), balanceStep) <= 0)
        throw InvalidConfigurationException((section+"::balance_step").c_str());
    if (cfg.get((section+"::sutter_logic").c_str(), shutterLogic) <= 0)
        throw InvalidConfigurationException((section+"::sutter_logic").c_str());
    if (cfg.get((section+"::backlight_compensation").c_str(), backlightCompensation) <= 0)
        throw InvalidConfigurationException((section+"::backlight_compensation").c_str());
    if (cfg.get((section+"::contrast").c_str(), contrast) <= 0)
        throw InvalidConfigurationException((section+"::contrast").c_str());
    if (cfg.get((section+"::sharpness").c_str(), sharpness) <= 0)
        throw InvalidConfigurationException((section+"::sharpness").c_str());
    
    if (!SUCCESS(unicap_enumerate_devices(NULL, &device, dev_nr)))
        EOUT("UNICAP: failed to enumerate device " << dev_nr);
    if (!SUCCESS(unicap_open(&handle, &device)))
        EOUT("UNICAP: failed to open device" << dev_nr);
    
    IOUT("UNICAP: opened device: " << device.identifier);
    
    int format_id = -1;
    for (int i=0; SUCCESS(unicap_enumerate_formats(handle, NULL, &format, i)); i++) {
        if (strstr(format.identifier, "YUYV") != NULL) format_id = i;
        IOUT("UNICAP: format " << i <<  " : " 
             << "identifier " << format.identifier  
             << " " << format.size.width
             << "x" << format.size.height);
    }
    if (format_id == -1)
        throw HardwareException("UNICAP: could not find any video devices.");
    IOUT("UNICAP: chose format id " << format_id);
    
    if (!SUCCESS(unicap_enumerate_formats(handle, NULL, &format, format_id)))
        EOUT("UNICAP: failed to enumerate formats");
    
    int subformat_id = -1;
    for (int i=0; i<format.size_count; i++) {
        if (format.sizes[i].width==size[0] && format.sizes[i].height==size[1]) {
            subformat_id = i;
            break;
        }
    }
    IOUT("UNICAP: chose subformat id " << subformat_id);
    
    format.size.width = format.sizes[subformat_id].width;
    format.size.height = format.sizes[subformat_id].height;
    format.buffer_type = UNICAP_BUFFER_TYPE_SYSTEM;
    
    if (!SUCCESS(unicap_set_format(handle, &format)))
        EOUT("UNICAP: failed to set format");
    unicap_property_t property;
    int property_framerate = -1, property_exposure = -1, property_temperature = -1, property_gain = -1;
    for (int i=0; SUCCESS(unicap_enumerate_properties(handle, NULL, &property, i)); i++)
        if (strstr(property.identifier, "frame") != NULL) {
            property.value = framerate;
            property_framerate = i;
            if (!SUCCESS(unicap_set_property(handle, &property)))
                EOUT("UNICAP: failed to set frame rate");
        } 
        else if (strstr(property.identifier, "Contrast") != NULL) {
            property.value = contrast;
            EOUT("setting contrast");
            if (!SUCCESS(unicap_set_property(handle, &property)))
                EOUT("UNICAP: failed to set contrast");
        }
        else if (strstr(property.identifier, "Sharpness") != NULL) {
            property.value = sharpness;
            EOUT("setting sharpness");
            if (!SUCCESS(unicap_set_property(handle, &property)))
                EOUT("UNICAP: failed to set sharpness");
        }

        else if (strstr(property.identifier, "Backlight Compensation") != NULL) {
            property.value = backlightCompensation;
            EOUT("setting backlight compensation");
            if (!SUCCESS(unicap_set_property(handle, &property)))
                EOUT("UNICAP: failed to set backlight compensation");
        }
        else if (soft_exposure && strstr(property.identifier, "Exposure") != NULL) {
            if (strstr(property.identifier, "Absolute") != NULL) {
                property_exposure = i;
                exposure = property;
                exp_min = property.range.min;
                exp_max = property.range.max;
                IOUT("Exposure ranges: "<<exp_min<<" "<<exp_max);
            } else if (strstr(property.identifier, "Auto") != NULL &&
                       strstr(property.identifier, "Priority") == NULL) {
                property.value = 1; // 1=off, 3=on
                if (!SUCCESS(unicap_set_property(handle, &property)))
	            EOUT("UNICAP: failed to set exposure to manual mode");
            }
        } else if (use_temperature && strstr(property.identifier, "Temperature") != NULL) {
            if (strstr(property.identifier, "Auto") != NULL) {
                property.value = 0;
                if (!SUCCESS(unicap_set_property(handle, &property)))
	            EOUT("UNICAP: failed to set whitebalance temperature to manual mode");
            } else {
                property_temperature = i;
                temperature = property;
                temp_min = property.range.min;
                temp_max = property.range.max;
                IOUT("Temperature ranges: "<<temp_min<<" "<<temp_max);
            }

        } else if (adjust_gain && strstr(property.identifier, "Gain") != NULL) {
            property_gain = i;
            gain = property;
            gain_min = property.range.min;
            gain_max = property.range.max;
            IOUT("Gain ranges: "<<gain_min<<" "<<gain_max);
        }

    if (property_framerate == -1)
        EOUT("UNICAP: failed to enumerate framerate");
    if (soft_exposure && property_exposure == -1)
        throw HardwareException("UNICAP: failed to enumerate exposure");
    if (use_temperature && property_temperature == -1)
        throw HardwareException("UNICAP: failed to enumerate temperature");
    if (adjust_gain && property_gain == -1)
        throw HardwareException("UNICAP: failed to enumerate gain");

    if (adjust_gain) setGain(def_gain);
    if (soft_exposure) setExposure(def_expo);
    if (use_temperature) setTemperature(def_temp);

/*    buffer = NULL;
    latest = NULL;
    unicap_register_callback(handle, UNICAP_EVENT_NEW_FRAME, (unicap_callback_t) new_frame_cb, this);
    unicap_start_capture(handle);
    usleep(100000);
//    unicap_stop_capture (handle);
    */


    // close device so that it can be used by V4L
    unicap_close (handle);

    frame_count = 0;
    autoExposureError = 0.0;
    autoExposureDelta = (exp_min-exp_max)/20.0;

#endif
}

#ifdef V4LIMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
void V4LImageSource::setTemperature(unsigned int temp)
{
  Time t2;
  temperature.value = temp;
    if (!SUCCESS(unicap_set_property(handle, &temperature)))
      EOUT("UNICAP: failed to set color temperature");
    IOUT("White balance: " << t2.elapsed_msec());
}

  void V4LImageSource::setExposure(unsigned int exp)
  {
    Time t2;
    exposure.value = oldShutter = exp;
    if (!SUCCESS(unicap_set_property(handle, &exposure)))
        EOUT("UNICAP: failed to set exposure");
    IOUT("Exposure: " << t2.elapsed_msec());
  }

void V4LImageSource::setGain(unsigned int g)
{
    Time t2;
    gain.value = oldGain = g;
    if (!SUCCESS(unicap_set_property(handle, &gain)))
        EOUT("UNICAP: failed to set exposure");
    IOUT("Gain: " << t2.elapsed_msec());
  }
#endif


void V4LImageSource::setupBuffer()
{
	v4l2_requestbuffers reqbuf;
	reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbuf.memory = V4L2_MEMORY_MMAP;
	reqbuf.count = 2;

	_imgbuf = new Tribots::ImageBuffer(
		V4LIMAGESOURCE_WIDTH,
		V4LIMAGESOURCE_HEIGHT,
		ImageBuffer::FORMAT_YUV422);

	if ( ioctl( _dev, VIDIOC_REQBUFS, &reqbuf ) == -1 )
	{
		cerr << "V4L: Device does not support mmap-streaming." << endl;
		return;
	}
	else if ( reqbuf.count < 2 )
	{
		cerr << "V4L: Failed to allocate memory." << endl;
		return;
	}
	else
	{
		_buffer.resize( reqbuf.count );
		_v4l2_mbuf.resize( reqbuf.count );

		for(unsigned int i = 0; i < reqbuf.count; i++ )
		{
			memset( &_v4l2_mbuf[ i ], 0, sizeof( v4l2_buffer ) );
			_v4l2_mbuf[ i ].type = reqbuf.type;
			_v4l2_mbuf[ i ].memory = V4L2_MEMORY_MMAP;
			_v4l2_mbuf[ i ].index = i;
			if ( ioctl( _dev, VIDIOC_QUERYBUF, &_v4l2_mbuf[ i ] ) == -1 )
			{
				cerr << "Failed to query _buffer." << endl;
			}
			_buffer[ i ] = static_cast< char* >( mmap( NULL, _v4l2_mbuf[ i ].length, PROT_READ | PROT_WRITE, MAP_SHARED, _dev, _v4l2_mbuf[ i ].m.offset ) );
			if ( !_buffer[ i ] )
			{
				cerr << "mmap failed." << endl;
				return;
			}
			if ( ioctl( _dev, VIDIOC_QBUF, &_v4l2_mbuf[ i ] ) == -1 )
			{
				cerr << "Failed to enqueue _buffer." << endl;
				return;	
			}
		}

		if ( ioctl( _dev, VIDIOC_STREAMON, &_v4l2_mbuf[ 0 ].type ) == -1)
		{
			cerr << "IOCTL STREAMON" << endl;
			return;
		}
		for(unsigned int i = 0; i < reqbuf.count; i++ )
		{
			if ( ioctl( _dev, VIDIOC_DQBUF, &_v4l2_mbuf[ i ] ) == -1 )
			{
				cerr << "IOCTL VIDIOC_DQBUF" << endl;
				return;
			}
		}
		if ( ioctl( _dev, VIDIOC_STREAMOFF, &_v4l2_mbuf[ 0 ].type ) == -1)
		{
			cerr << "IOCTL STREAMOFF" << endl;
			return;
		}
		for(unsigned  int i = 0; i < reqbuf.count; i++ )
		{
			if ( ioctl( _dev, VIDIOC_QBUF, &_v4l2_mbuf[ i ] ) == -1 )
			{
				cerr << "Failed to enqueue _buffer." << endl;
				return;
			}
		}
		if ( ioctl( _dev, VIDIOC_STREAMON, &_v4l2_mbuf[ 0 ].type ) == -1)
		{
			cerr << "IOCTL STREAMON" << endl;
			return;
		}
	}
}
/*
void VisionV4L2::grab( char* data, unsigned int* dataSize, unsigned int maxDataSize )
{
	rec::core_lt::memory::ByteArrayConst ba;

	struct v4l2_buffer* vb = &_v4l2_mbuf[ _frame ];

	if( 0 == ioctl( _dev, VIDIOC_DQBUF, vb ) )
	{
		*dataSize = vb->bytesused;
		if( *dataSize > maxDataSize )
		{
			throw rec::core_lt::Exception( "VisionV4L2::grab image size to large" );
		}

		memcpy( (void*)data, (const void*)_buffer[ _frame ], *dataSize );
	}
	else
	{
		throw rec::core_lt::Exception( "IOCTL VIDIOC_DQBUF" );
	}

	if ( ioctl( _dev, VIDIOC_QBUF, vb ) == -1 )
	{
		throw rec::core_lt::Exception( "IOCTL VIDIOC_QBUF" );
	}

	_frame = ( _frame + 1 ) % _v4l2_mbuf.size();
}
*/

void V4LImageSource::setupFormat(int width, int height)
{
	v4l2_format fmt;

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if ( -1 == ioctl( _dev, VIDIOC_G_FMT, &fmt ) )
	{
		cerr << "Failed to get format" << endl;
		return;
	}
        cout << fmt.fmt.pix.pixelformat << endl;

	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	std::cout << "Using V4L version 2. " << width << " , " << height << std::endl;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	if( -1 == ioctl( _dev, VIDIOC_S_FMT, &fmt ) )
	{
		cerr << "Failed to set format"  << endl;
                system("dov4l -p YUYV -s 640,480");
	}

	_width = width;
	_height = height;
}

}

