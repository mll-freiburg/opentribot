#include "tribotsv4l.h"
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

#include "../../../Fundamental/Time.h"

#include <cstring>

#include "AssiLib.h"

#define EOUT( __x__ ) std::cerr << "      #Error: [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "      \n"
#define IOUT( __x__ ) std::cerr << "      #INFO : [" << __PRETTY_FUNCTION__ << "]: " << __x__ << "      \n"

using namespace std;
using namespace Tribots;

// Folgendes wird zur Anmeldung bei der Factory benoetigt:
namespace
{
  class Builder:public Tribots::ImageSourceBuilder
  {
  public:
    Builder ()
    {
      Tribots::ImageSourceFactory::
	get_image_source_factory ()->sign_up (string ("PS3"), this);
    }
    Tribots::ImageSource * get_image_source (const std::string &,
					     const Tribots::ConfigReader &
					     reader,
					     const std::string & section)
      throw (Tribots::TribotsException, std::bad_alloc)
    {
      return new Tribots::PS3ImageSource (reader, section);
    }
  };
  Builder the_builder;
}


namespace Tribots
{

  PS3ImageSource::PS3ImageSource (const ConfigReader & cfg,
				  const std::string & section,
				  bool) throw (HardwareException,
					       InvalidConfigurationException)
  {
    red_balance=-1;
    blue_balance=-1;
    exposure=-1;
    autogain=-1;
    main_gain=-1;
    hflip=-1;
    vflip=-1;
    sharpness=-1;

    framecount=0;
    red_balance =
      CONFIG_GETVAR ("PS3Eye.cfg", "red_balance", 128, "Red balance", 0, 255);
    blue_balance =
      CONFIG_GETVAR ("PS3Eye.cfg", "blue_balance", 128, "Blue balance", 0,
		     255);
 

setBalanceArea(320,240,20,20);
    bAX =
      CONFIG_GETVAR ("PS3Eye.cfg", "balance_x", 320, "balanceArea", 0, 640);
    bAY =
      CONFIG_GETVAR ("PS3Eye.cfg", "balance_y", 240, "BalanceArea", 0,
		     480);
    bAWidth =
      CONFIG_GETVAR ("PS3Eye.cfg", "balance_w", 20, "balanceArea", 0, 640);
    bAHeight =
      CONFIG_GETVAR ("PS3Eye.cfg", "balance_h", 20, "BalanceArea", 0,
		     480);

	imageWidth=640;
	imageHeight=480;
   shutterMax=255;
   shutterMin=0;
int shutterLogic=1;
	autoExposureError=0;
   	autoExposureDelta=(shutterMax-shutterMin)/20;


    /* initialize PS3 device */

    string device;
    if (cfg.get ((section + "::device_name").c_str (), device) <= 0)
      throw
	InvalidConfigurationException ((section + "::device_name").c_str ());

      _dev = open (device.c_str(), O_RDWR, 0);
    if (-1 == _dev){
	cout <<"while trying to open "<<device.c_str()<<endl; 
     throw HardwareException ("PS3Eye: Failed to open device.");
}
    struct v4l2_capability v4l2cap;
    if (ioctl (_dev, VIDIOC_QUERYCAP, &v4l2cap) == -1)
      throw HardwareException ("V4L: Not a V4L2 device");

    if (false == (V4L2_CAP_VIDEO_CAPTURE & v4l2cap.capabilities))
      throw HardwareException ("V4L: Does not support VIDEO_CAPTURE");

    /* prepare buffers */

      setupBuffer ();
      setupFormat (PS3IMAGESOURCE_WIDTH, PS3IMAGESOURCE_HEIGHT);
      _frame = 0;

    if (cfg.get ((section + "::delay").c_str (), delay) <= 0)
      throw InvalidConfigurationException ((section + "::delay").c_str ());





      controls.set_device (device.c_str ());
      setCameraParameters ();

//	_imgbuf=new ImageBuffer();

  }

  PS3ImageSource::~PS3ImageSource () throw ()
  {
    // TODO anything to close? throws errors at quit
  }

  ImageBuffer PS3ImageSource::getImage () throw (ImageFailed)
  { 
    framecount++;
    Time t2;
	t2.update();
    setCameraParameters ();
    struct v4l2_buffer *vb = &_v4l2_mbuf[_frame];

    if (0 == ioctl (_dev, VIDIOC_DQBUF, vb))
      {
	 //cout << "using buffer " << _imgbuf->size << endl;

	_imgbuf->timestamp.update ();
	_imgbuf->timestamp.add_msec (delay);
	_imgbuf->size = vb->bytesused;
	 cout << "using buffer " << vb->bytesused << endl;
	_imgbuf->buffer = (unsigned char *) realloc ((void *) _imgbuf->buffer,
					     _imgbuf->size);

	memcpy (_imgbuf->buffer, _buffer[_frame], vb->bytesused);
	int length = vb->bytesused;
	uint16_t *words = (uint16_t *) _imgbuf->buffer;
	while (length >= 2)
	  {
	    *words = ((*words >> 8) & 0xff) | ((*words & 0xff) << 8);
	    words++;
	    length -= 2;
	  }
      }
    else
      {
	cerr << "IOCTL VIDIOC_DQBUF: " << strerror (errno) << endl;
	return NULL;
      }

    if (ioctl (_dev, VIDIOC_QBUF, vb) == -1)
      {
	cerr << "IOCTL VIDIOC_QBUF" << endl;
	return NULL;
      }

    _frame = (_frame + 1) % _v4l2_mbuf.size ();
    //cout <<"Image returnedt " << t2.elapsed_usec ()<<endl;
    return *_imgbuf;
    



  }


void PS3ImageSource::doSoftwareExposure(){

  if (framecount % 3 != 0) {
    return;
  }

  stringstream str;
  Time timer; timer.update();
  unsigned int oldShutter = 0;
  unsigned int newshutter = 0;
  unsigned int oldGain = 0;
  unsigned int newgain = 0;

    oldShutter = exposure;
    newshutter = oldShutter;
    oldGain = main_gain;
    newgain = oldGain;
int shutterLogic=0;
// USing color Coding yuv422
int gainMin=0;
int gainMax=63;
  int iSize = (int) (imageWidth * imageHeight * 2);
   
      int start=0;
      int step=12;
      int offsetU=1;
      int offsetV=2;
      double bytePerPixel = 3;
   
  double average  = 127.;



	long value = 0; int count = 0;
	for (int c = start; c < iSize; c+=step) {
	  //value += ((unsigned char*)camera.capture_buffer)[c];
	  value += ((unsigned char*)_imgbuf->buffer)[c];
	  ++count;
	}
	average  = value / (double)count;
 
  double normDelta = (shutterMax-shutterMin)*1.0f/20;
  double error = (static_cast<double>(average)-static_cast<double>(exposure));
  if (abs(error)>=3) {
    if (error*autoExposureError>0) { 
      double maxDelta = (shutterMax-shutterMin)/10;
      autoExposureDelta*=1.2;
      if (autoExposureDelta>maxDelta)
        autoExposureDelta=maxDelta;
    } else if (error*autoExposureError<0) {
      autoExposureDelta*=0.5;
      if (autoExposureDelta<0.5)
        autoExposureDelta=0.5;
    }
    if (abs(error-autoExposureError)>20 && autoExposureDelta<normDelta)
      autoExposureDelta=normDelta;
    autoExposureError=error;
    int dShutter=0;
    if (error<0)
      dShutter = static_cast<int>(floor((shutterLogic >= 0 ? +1 : -1)*autoExposureDelta+0.5));
    else if (error>0)
      dShutter = - static_cast<int>(floor((shutterLogic >= 0 ? +1 : -1)*autoExposureDelta+0.5));
    if (static_cast<int>(oldShutter)>=-dShutter)
      newshutter = oldShutter-dShutter;
    else
      newshutter=0;
  } else {
    newshutter = oldShutter;
    autoExposureDelta*=0.5;
    if (autoExposureDelta<0.5)
      autoExposureDelta=0.5;
  }

  if (newshutter<shutterMin) {
    newshutter=shutterMin;
    if (autoExposureDelta>normDelta)
      autoExposureDelta=normDelta;
  }
  if (newshutter>shutterMax) {
    newshutter=shutterMax;
    if (autoExposureDelta>normDelta)
      autoExposureDelta=normDelta;
  }
  if ((newshutter >= shutterMax && shutterLogic>=0) || (newshutter<=shutterMin && shutterLogic<0)) {   // raise gain, still to dark
    newgain = (unsigned int) (ceil(oldGain * 1.08+0.01));
  }
  else if (((newshutter < 0.2*shutterMin+0.8*shutterMax) && (shutterLogic>=0)) || ((newshutter > 0.8*shutterMin+0.2*shutterMax) && (shutterLogic<0))) {  // could decrease gain again
    int newGainInt = (int) (floor(oldGain * .97-0.01));
    newgain = (newGainInt>=0 ? newGainInt : 0);
  }




    if (newshutter != exposure)
      {
        newshutter = newshutter < shutterMin ? shutterMin : 
                                     newshutter > shutterMax ? shutterMax : newshutter;
	exposure = newshutter;
   
	controls.set_control ("exposure", exposure);
      }

	
    if (newgain != main_gain)
      {
       newgain = newgain < gainMin ? gainMin : newgain > gainMax ? gainMax : newgain;
	main_gain = newgain;    
	controls.set_control ("main_gain", main_gain);
      }



  cout << (timer.elapsed_usec() / 1000.)<<endl;;
  //JMESSAGE(str.str().c_str());

}


 void PS3ImageSource::setBalanceArea(int x, int y, int w, int h){
	bAX=x;
	bAY=y;
	bAWidth=w;
	bAHeight=h;
      CONFIG_SETVAR ("PS3Eye.cfg", "balance_x", x);
      CONFIG_SETVAR ("PS3Eye.cfg", "balance_y", y);
      CONFIG_SETVAR ("PS3Eye.cfg", "balance_w", w);
      CONFIG_SETVAR ("PS3Eye.cfg", "balance_h", h);
cout << "Yo hab die balance area gesetzt"<<endl;



}
void PS3ImageSource::getBalanceArea(int *x, int *y, int *w, int *h) 
{
	*x=bAX;
	*y=bAY;
	*w=bAWidth;
	*h=bAHeight;


}



void PS3ImageSource::doSoftwareBalance(){

  if (framecount % 3 != 0) {
    return;
  }

  stringstream str;
  Time timer; timer.update();


  unsigned int oldU, oldV;
  oldU=blue_balance;
  oldV=red_balance;
  unsigned int u=oldU, v=oldV;
  str << "doSoftwareBalance: " << (timer.elapsed_usec() / 1000.) << " ";

  int iSize = (int) (imageWidth * imageHeight * 2);
   
  double average  = 127.;
  double averageU = 127.;
  double averageV = 127.;
   
      int start=0;
      int step=12;
      int offsetU=1;
      int offsetV=2;
      double bytePerPixel = 3;


	//      case IIDC::YUV422: 
	start = 1; step = 1 *  8; offsetU = -1; offsetV = 1; bytePerPixel = 2;


        int balanceAreaX = bAX % 4 ? bAX + 4 - (bAX % 4) : bAX;
        int lineOffset = (int)(bytePerPixel * imageWidth);
        int balanceAreaStart = (int)
          (bytePerPixel * balanceAreaX + lineOffset * bAY + start);
        int pos = 0;

        long uV=0, vV=0; int count=0;
        for (int y = 0; y < bAHeight; y++) {
          double w2 = 0.5*static_cast<double>(bAWidth);
          double h2 = 0.5*static_cast<double>(bAHeight);
          double yd = static_cast<double>(y);
          double r = 2*yd*h2-yd*yd;
          double rr = (r>0 ? std::sqrt(r)*w2/h2 : 0.0);
          int x0=static_cast<int>(w2-rr);
          int x1=static_cast<int>(w2+rr);
          for (int x = x0; x <= x1; x+=4) {
            pos = (int) (balanceAreaStart + y * lineOffset + x * bytePerPixel);
            uV += ((unsigned char*)_imgbuf->buffer)[pos+offsetU];
            vV += ((unsigned char*)_imgbuf->buffer)[pos+offsetV];
            ++count;
        }
        averageU = uV / (double)count;
        averageV = vV / (double)count;
      }

  double uTmp = oldU * (1. - (averageU-127.) / 255);
  double vTmp = oldV * (1. - (averageV-127.) / 255);
  u = (unsigned int) (uTmp > oldU ? floor(uTmp) : ceil(uTmp));
  v = (unsigned int) (vTmp > oldV ? floor(vTmp) : ceil(vTmp));

//  cout <<"avg u "<<averageU<< "oldU"<<oldU<<"  "<<u <<endl;
//  cout <<"avg v "<<averageV<< "oldV"<<oldV<<"  "<<v<<endl;

 
	blue_balance = u;
	controls.set_control ("blue_balance", blue_balance);
	red_balance = v;
	controls.set_control ("red_balance", red_balance);
	    





}



  void PS3ImageSource::setCameraParameters ()


    throw (InvalidConfigurationException)
  {
  //return;

    Time t2;

    int new_red_balance;
    int new_blue_balance;
    int new_exposure;
    int new_autogain;
    int new_main_gain;
    int new_hflip;
    int new_vflip;
    int new_sharpness;
    bool software_balance;

    bool software_exposure;

    software_balance=CONFIG_GETVAR ("PS3Eye.cfg", "software_whitebalance", 0, "Software White Balance", 0, 1);
    new_red_balance =
      CONFIG_GETVAR ("PS3Eye.cfg", "red_balance", 128, "Red balance", 0, 255);
    new_blue_balance =
      CONFIG_GETVAR ("PS3Eye.cfg", "blue_balance", 128, "Blue balance", 0,
		     255);
    new_exposure =
      CONFIG_GETVAR ("PS3Eye.cfg", "exposure", 140, "exposure/Shutter", 0,
		     255);
    new_autogain =
      CONFIG_GETVAR ("PS3Eye.cfg", "autogain", 0, "Autogain", 0, 1);
    new_main_gain =
      CONFIG_GETVAR ("PS3Eye.cfg", "main_gain", 20, "Main Gain", 0, 63);


    new_hflip = CONFIG_GETVAR ("PS3Eye.cfg", "hflip", 0, "Hflip", 0, 1);
    new_vflip = CONFIG_GETVAR ("PS3Eye.cfg", "vflip", 0, "Vflip", 0, 1);
    new_sharpness =
      CONFIG_GETVAR ("PS3Eye.cfg", "sharpness", 128, "Sharpness", 0, 255);
    new_sharpness =
      CONFIG_GETVAR ("PS3Eye.cfg", "light_frequency_filter", 1, "Light frequency", 0, 1);




    if (software_balance){
if (framecount !=0)	doSoftwareBalance();

	}
	else{
    if (new_blue_balance != blue_balance)
      {
	blue_balance = new_blue_balance;
	controls.set_control ("blue_balance", blue_balance);
}      
    if (new_red_balance != red_balance)
      {
	red_balance = new_red_balance;
	controls.set_control ("red_balance", red_balance);
      };
	    

	}

    software_exposure=CONFIG_GETVAR ("PS3Eye.cfg", "software_exposure", 0, "Software Exposure", 0, 1);
    if (software_exposure){
	if (framecount!=0)doSoftwareExposure();}

     else {

    if (new_autogain != autogain)
      {
	autogain = new_autogain;
	cout << "Setting autogain to" <<autogain<<endl;
	controls.set_control ("autogain", autogain);
      };
    if(autogain==0){
    if (new_exposure != exposure)
      {
	exposure = new_exposure;
	controls.set_control ("exposure", exposure);
      }

	}
    if (new_main_gain != main_gain)
      {
	main_gain = new_main_gain;
	controls.set_control ("main_gain", main_gain);
      }






}



    if (new_hflip != hflip)
      {
	hflip = new_hflip;
	controls.set_control ("hflip", hflip);
      }
    if (new_vflip != vflip)
      {
	vflip = new_vflip;
	controls.set_control ("vflip", vflip);
      };
    if (new_sharpness != sharpness)
      {
	sharpness = new_sharpness;
	controls.set_control ("sharpness", sharpness);
      }


//    set_ctrls[std::string(value, (equal - v     alue))] = equal + 1;


//    cout <<"Camera Parameter set " << t2.elapsed_usec ()<<endl;


  }

#ifdef PS3IMAGESOURCE_USE_UNICAP_FOR_PARAMETERS
  void PS3ImageSource::setTemperature (unsigned int temp)
  {
    Time t2;
    temperature.value = temp;
    if (!SUCCESS (unicap_set_property (handle, &temperature)))
      EOUT ("UNICAP: failed to set color temperature");
    IOUT ("White balance: " << t2.elapsed_msec ());
  }

  void PS3ImageSource::setExposure (unsigned int exp)
  {
    Time t2;
    exposure.value = oldShutter = exp;
    if (!SUCCESS (unicap_set_property (handle, &exposure)))
      EOUT ("UNICAP: failed to set exposure");
    IOUT ("Exposure: " << t2.elapsed_msec ());
  }

  void PS3ImageSource::setGain (unsigned int g)
  {
    Time t2;
    gain.value = oldGain = g;
    if (!SUCCESS (unicap_set_property (handle, &gain)))
      EOUT ("UNICAP: failed to set exposure");
    IOUT ("Gain: " << t2.elapsed_msec ());
  }
#endif


  void PS3ImageSource::setupBuffer ()
  {
    v4l2_requestbuffers reqbuf;
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    reqbuf.count = 2;

    _imgbuf = new Tribots::ImageBuffer (PS3IMAGESOURCE_WIDTH,
					PS3IMAGESOURCE_HEIGHT,
					ImageBuffer::FORMAT_YUV422);

    if (ioctl (_dev, VIDIOC_REQBUFS, &reqbuf) == -1)
      {
	cerr << "V4L: Device does not support mmap-streaming." << endl;
	return;
      }
    else if (reqbuf.count < 2)
      {
	cerr << "V4L: Failed to allocate memory." << endl;
	return;
      }
    else
      {
	_buffer.resize (reqbuf.count);
	_v4l2_mbuf.resize (reqbuf.count);

	for (unsigned int i = 0; i < reqbuf.count; i++)
	  
	  {
	    memset (&_v4l2_mbuf[i], 0, sizeof (v4l2_buffer));
	    _v4l2_mbuf[i].type = reqbuf.type;
	    _v4l2_mbuf[i].memory = V4L2_MEMORY_MMAP;
	    _v4l2_mbuf[i].index = i;
	    if (ioctl (_dev, VIDIOC_QUERYBUF, &_v4l2_mbuf[i]) == -1)
	      
	      {
		cerr << "Failed to query _buffer." << endl;
	      }
	    _buffer[i] =
	      static_cast <
	      char
	      *>(mmap
		 (NULL, _v4l2_mbuf[i].length, PROT_READ | PROT_WRITE,
		  MAP_SHARED, _dev, _v4l2_mbuf[i].m.offset));
	    if (!_buffer[i])
	      
	      {
		cerr << "mmap failed." << endl;
		return;
	      }
	    if (ioctl (_dev, VIDIOC_QBUF, &_v4l2_mbuf[i]) == -1)
	      
	      {
		cerr << "Failed to enqueue _buffer." << endl;
		return;
	      }
	  }

	if (ioctl (_dev, VIDIOC_STREAMON, &_v4l2_mbuf[0].type) == -1)
	  
	  {
	    cerr << "IOCTL STREAMON" << endl;
	    return;
	  }
	for (unsigned int i = 0; i < reqbuf.count; i++)
	  
	  {
	    if (ioctl (_dev, VIDIOC_DQBUF, &_v4l2_mbuf[i]) == -1)
	      
	      {
		cerr << "IOCTL VIDIOC_DQBUF" << endl;
		return;
	      }
	  }
	if (ioctl (_dev, VIDIOC_STREAMOFF, &_v4l2_mbuf[0].type) == -1)
	  
	  {
	    cerr << "IOCTL STREAMOFF" << endl;
	    return;
	  }
	for (unsigned int i = 0; i < reqbuf.count; i++)
	  
	  {
	    if (ioctl (_dev, VIDIOC_QBUF, &_v4l2_mbuf[i]) == -1)
	      
	      {
		cerr << "Failed to enqueue _buffer." << endl;
		return;
	      }
	  }
	if (ioctl (_dev, VIDIOC_STREAMON, &_v4l2_mbuf[0].type) == -1)
	  
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
  void PS3ImageSource::setupFormat (int width, int height)
  {
    v4l2_format fmt;

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (-1 == ioctl (_dev, VIDIOC_G_FMT, &fmt))
      
      {
	cerr << "Failed to get format" << endl;
	return;
      }
    cout << fmt.fmt.pix.pixelformat << endl;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    std::
      cout << "Using V4L version 2. " << width << " , " << height <<
      std::endl;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    if (-1 == ioctl (_dev, VIDIOC_S_FMT, &fmt))
      {
	cerr << "Failed to set format" << endl;
	system ("dov4l -p YUYV -s 640,480");
      }
    _width = width;
    _height = height;
  }
}
