
#include "CamDriver.h"
#include "../../Fundamental/stringconvert.h"
#include "../../Fundamental/random.h"
#include "../../Structures/Journal.h"
#include <iostream>
#include <unistd.h>
#include <libraw1394/csr.h>
#include <signal.h>
#include <sys/time.h>
#include <sstream>
#include <map>
#include <cmath>
#include <sstream>

#ifndef INFO_OUT
#define INFO_OUT cerr
#endif

#ifndef ERROR_OUT
#define ERROR_OUT cerr
#endif


using namespace std;
using namespace Tribots;


namespace {
  std::map<IIDC::CameraFeature, unsigned int> init_libdc_feature_mapping () {
    std::map<IIDC::CameraFeature, unsigned int> dest;
    dest[IIDC::brightness]=FEATURE_BRIGHTNESS;
    dest[IIDC::exposure]=FEATURE_EXPOSURE;
    dest[IIDC::sharpness] = FEATURE_SHARPNESS;
    dest[IIDC::whiteBalance] = FEATURE_WHITE_BALANCE;
    dest[IIDC::hue] = FEATURE_HUE;
    dest[IIDC::saturation] = FEATURE_SATURATION;
    dest[IIDC::gamma] = FEATURE_GAMMA;
    dest[IIDC::shutter] = FEATURE_SHUTTER;
    dest[IIDC::gain] = FEATURE_GAIN;
    dest[IIDC::filter] = FEATURE_OPTICAL_FILTER;
    return dest;
  }
  std::map<IIDC::ColorCoding, unsigned int> init_libdc_colorCoding_mapping() {
    std::map<IIDC::ColorCoding, unsigned int> dest;
    dest[IIDC::YUV411] = COLOR_FORMAT7_YUV411;
    dest[IIDC::YUV422] = COLOR_FORMAT7_YUV422;
    dest[IIDC::YUV444] = COLOR_FORMAT7_YUV444;
    dest[IIDC::RGB8] = COLOR_FORMAT7_RGB8;
    dest[IIDC::RGB16] = COLOR_FORMAT7_RGB16;
    dest[IIDC::Mono8] = COLOR_FORMAT7_MONO8;
    dest[IIDC::Mono16] = COLOR_FORMAT7_MONO16;
    return dest;
  }
  std::map<IIDC::CameraFeature, std::string> init_featurename_mapping () {
    std::map<IIDC::CameraFeature, std::string> dest;
    dest[IIDC::brightness]=string("brightness");
    dest[IIDC::exposure]=string("exposure");
    dest[IIDC::sharpness] = string("sharpness");
    dest[IIDC::whiteBalance] = string("whiteBalance");
    dest[IIDC::hue] = string("hue");
    dest[IIDC::saturation] = string("saturation");
    dest[IIDC::gamma] = string("gamma");
    dest[IIDC::shutter] = string("shutter");
    dest[IIDC::gain] = string("gain");
    dest[IIDC::filter] = string("filter");
    return dest;
  }
  std::map<IIDC::CameraFeatureMode, std::string> init_featuremodename_mapping () {
    std::map<IIDC::CameraFeatureMode, std::string> dest;
    dest[IIDC::featureUnavailable]=string("unavailable");
    dest[IIDC::featureOff]=string("off");
    dest[IIDC::featureMan] = string("man");
    dest[IIDC::featureAuto] = string("auto");
    return dest;
  }
  
  /*const*/ std::map <IIDC::CameraFeature, unsigned int> libdc_feature_mapping (init_libdc_feature_mapping());
  /*const*/ std::map <IIDC::ColorCoding, unsigned int> libdc_colorCoding_mapping (init_libdc_colorCoding_mapping());
  /*const*/ std::map <IIDC::CameraFeature, std::string> featurename_mapping (init_featurename_mapping());
  /*const*/ std::map <IIDC::CameraFeatureMode, std::string> featuremodename_mapping (init_featuremodename_mapping());

  int delay_iidc_commands = 50000;  // soll-Wartezeit zwischen zwei IIDC-Kommandos (z.B.Feature-Zugriff)
}

/* Initialize static variables */

vector<CamDriver*> CamDriver::multicam = vector<CamDriver*>();
unsigned int CamDriver::bandwidth_used = 0;
unsigned int CamDriver::reset_count = 0;
int CamDriver::numNodes = 0;
int CamDriver::num_ports = 0;
std::vector<CamDriver::DCInfo> CamDriver::cameranodes;


/**
 * Signal handler for the timer "trick". Does nothing, but enables us to
 * return from the dc1394_dma_capture library call, even if the wait modus
 * is used.
 */
static
void sighandler_nop(int) {   //  blocking call
  ERROR_OUT << "CamDriver: sighandler_nop called. Call to "
	    << "dc1394_dma_single_capture() "
	    << "did not return." << endl;
}


/**
 * Reads a quadlet from a given ieee1394 port. Used to detect bus resets.
 * Inspired by the cooked_read of coriander and gscanbus.
 */
static
void* dummy_read(void* handle)
{
  quadlet_t quadlet;
  int retval = -1;

  retval = raw1394_read(*(raw1394handle_t*)handle,
			0xffc0 |
			raw1394_get_local_id(*(raw1394handle_t*)handle),
			CSR_REGISTER_BASE + CSR_CYCLE_TIME, 4,
			&quadlet);

  return NULL;
}


/**
 * Static C bus reset hanlder passed to the raw1394 system.
 * Calls bus_reset_handler_static, which passes the reset to the right camera instance.
 *
 * @returns Return value of the instance resetter.
 */
namespace Tribots
{
int __iidc_bus_reset_handler_static( raw1394handle_t handle, unsigned int generation )
{
  return CamDriver::bus_reset_handler_static(handle, generation);
}
};


int CamDriver::bus_reset_handler_static(raw1394handle_t handle, unsigned int generation ) throw ()
{
  // increase reset count
  ++reset_count;

  INFO_OUT << "#INTERUPT : Reset handler called. Count: "<<reset_count << endl;
   // if this was the first reset count: perform reset. Otherwise do nothing (and jump back to
   //  the already executed reset.
  if(reset_count==1){
     usleep(1000*200);  // safety delay

     for(unsigned int i=0; i<multicam.size(); i++)
       multicam[i]->deinit();

     // no cameras should be running anymore: no bandwidth used
     bandwidth_used = 0;

     // This loop is repeated if more than one bus reset was detected, to make
     // sure the latest changes (e.g. a camera has been replugged) are taken
     // into account.
     // If all cameras have been restarted, however, the loop is broken.
     while(reset_count>0){
        //Remember how often the bus has been resettet.
        raw1394_update_generation(handle, generation);

        // Check for cameras
        searchCameraNodes ();
        INFO_OUT << "reset_handler_static: "<< cameranodes.size() << " cameras found." << endl;

        --reset_count;
     }
     return 1;
  }

  usleep (200*1000);  // safety delay
  return 0;
}

std::vector<std::string> CamDriver::getCameraUIDs () throw (HardwareException) {
  if (cameranodes.empty())
    searchCameraNodes ();
  std::vector<std::string> uids (cameranodes.size());
  for (unsigned int i=0; i<uids.size(); i++)
    uids[i]=cameranodes[i].uid;
  cameranodes.clear();
  return uids;
}

void CamDriver::searchCameraNodes () throw (HardwareException){
  cameranodes.clear();
  
  raw1394handle_t dc_handle;
  nodeid_t *camera_nodes;
  raw1394handle_t raw_handle;

  //Get handle to the firewire bus.
  raw_handle = raw1394_new_handle();
  if (raw_handle == NULL) {
    ERROR_OUT << "Unable to create raw1394_handle. Did you load the drivers?";
    throw HardwareException(__FILE__
        ": Could not create handle during setup");
  }
  numNodes = raw1394_get_nodecount(raw_handle);
  num_ports = raw1394_get_port_info(raw_handle, NULL, 0);
  raw1394_destroy_handle(raw_handle);

  // for all ports look for cameras
  for (int p = 0; p < num_ports; ++p) {
    int camCount;
    dc_handle = dc1394_create_handle(p);
    if (dc_handle == NULL) {
      ERROR_OUT << "\nUnable to aquire a raw1394 handle\n\n"
          << "Please check \n"
          << "  - if the kernel modules video1394, "
          << "`ieee1394',`raw1394' and `ohci1394'"
          << " are loaded \n"
          << "  - if you have read/write access to /dev/raw1394\n\n";
      throw HardwareException(__FILE__
          ": Could not create handle during setup");
    }
    // get number of cameras on the current port.
    // camera_nodes is the nodeid_t* array on the current port, camCount the number of cameras found.
    // the "1" prints the camera infos on std::outcamera_node
    camera_nodes = dc1394_get_camera_nodes(dc_handle, &camCount, 1);

    // for all cameras found on that port put them into 'cameranodes'
    for(int i = 0; i < camCount; ++i){
      DCInfo new_node;
      new_node.port = p;
      new_node.node = camera_nodes[i];
      new_node.uid = getCameraUID (dc_handle, camera_nodes[i]);
      new_node.channel = i+1;
      cameranodes.push_back (new_node);
    }
    dc1394_destroy_handle(dc_handle);
    dc1394_free_camera_nodes(camera_nodes);      //frees camera nodes array
  } // end for all ports
}

const CamDriver::DCInfo* CamDriver::getCameraNode (const std::string& uid) throw (){
  for (unsigned int i=0; i<cameranodes.size(); i++)
    if (cameranodes[i].uid==uid)
      return &cameranodes[i];
  if (uid=="" && cameranodes.size()>0)
    return &cameranodes[0];  // first camera on the bus, if uid="" demanded
  return NULL;  // not found
}

IIDC::CameraError CamDriver::restartCamera () throw () {
  INFO_OUT << "Start/Restart camera with UID=" << uid << " ... \n";
  if (handle) deinit ();
  
  const DCInfo* dci = getCameraNode (uid);
  if (!dci) {
    INFO_OUT << "camera not found." << endl;
    return IIDC::cameraNotFound;
  }
  if (dci->node == numNodes-1) {
    INFO_OUT << "camera became root node -> do bus reset\n";
    raw1394_reset_bus (handle);
    return IIDC::unknownReason;
  }
  
  uid = dci->uid;
  initDone=false;
  dmaInitDone=false;
  startDone=false;
  
  port = dci->port;
  camera.node = dci->node;
  channel = dci->channel;
  handle = dc1394_create_handle (port);
  if (handle==NULL) {
    INFO_OUT << "got no handle\n";
    return IIDC::unknownReason;
  }
    
  IIDC::CameraError initerr = init ();
  if (initerr!=IIDC::successful) return initerr;
  IIDC::CameraError starterr = startTransmission ();
  if (starterr!=IIDC::successful) {
    deinit ();
    return starterr;
  }
  
  setSavedCameraFeatures ();
  
  INFO_OUT << "restart successfully done." << endl;
  
  return IIDC::successful;
}

CamDriver* CamDriver::getCamera( const char* device_name,
       int,       // (port) For backward compatibility only
       const std::string& mode,
       const std::string& uid,
       bool blocking,
       int delay) throw (HardwareException)
       
{
  CamDriver* new_cam = new CamDriver (device_name, uid, blocking, delay);
  
  if (!revealCameraMode (new_cam->cameraFormat, new_cam->colorCoding, new_cam->framerate, new_cam->imageWidth, new_cam->imageHeight, new_cam->offsetX, new_cam->offsetY, mode)) {
    ERROR_OUT << "Invalid camera mode description." << endl;
    throw HardwareException(__FILE__
        ": Invalid camera mode description.");
  }
  new_cam->bAWidth = new_cam->imageWidth;    // TODO: Ist das so schoen???
  new_cam->bAHeight = new_cam->imageHeight; 
  
  if (cameranodes.empty())
    searchCameraNodes ();

  IIDC::CameraError err = new_cam->restartCamera();
  if (err!=IIDC::successful) {
    ERROR_OUT << "failed in initializing and starting camera" << endl;
    throw HardwareException(__FILE__
        ": Error initializing and starting camera.");
  }
  multicam.push_back (new_cam);
  return new_cam;
}

void CamDriver::destroyAllCamera() throw ()
{
  for (unsigned int i=0; i<multicam.size(); i++)
    delete multicam[i];
  multicam.clear();
  cameranodes.clear();
}

void CamDriver::destroyCamera(CamDriver* that) throw ()
{
  for (unsigned int i=0; i<multicam.size(); i++) {
    if (multicam[i]==that) {
     delete multicam[i];
     multicam.erase (multicam.begin()+i);
     break;
    }
  }
  if (multicam.size()==0)
    destroyAllCamera(); // beim Loeschen der letzten Kamera auch die Kameraknoten loeschen
}


CamDriver::CamDriver (const char* device_name, const std::string& uid, bool blocking, int delay) throw ()
  : doSoftExposure(false), doSoftWhiteBalance(false), device_name (device_name), uid(uid), isBlocking(blocking), camera_delay (delay), bandwidth_cam(0), bAX(0), bAY(0), bAWidth(0), bAHeight(0)
{
   initDone=false;
   dmaInitDone=false;
   startDone=false;
   handle = 0;
   frameCount = 0;
}

void CamDriver::setBalanceArea(int x, int y, int w, int h) 
{
  bAX = x;
  bAY = y;
  bAWidth = w;
  bAHeight = h;
}

void CamDriver::getBalanceArea(int *x, int *y, int *w, int *h) const
{
  *x = bAX;
  *y = bAY;
  *w = bAWidth;
  *h = bAHeight;
}

IIDC::CameraError CamDriver::init() throw () {
   ///////////////// reset mechanism  //////////////////
   raw1394_set_bus_reset_handler(handle, __iidc_bus_reset_handler_static);
   // register a handler that is called, whenever the bus is resetted

   if (dc1394_init_camera (handle, camera.node) == DC1394_FAILURE)
   {
      ERROR_OUT << "Error initializing camera on node " << camera.node << " with uid " << uid << endl;
      return IIDC::initFailed;
   }

   // set trigger mode
   if (dc1394_set_trigger_mode (handle, camera.node, TRIGGER_MODE_0) != DC1394_SUCCESS)
   {
      INFO_OUT << "unable to set camera trigger mode\n";
   }

   // Load camera settings stored in (hardware-) channel 1 of the camera
   usleep(100000);
//   dc1394_memory_load (handle, camera.node, (unsigned int) 1);

   initDone=true;   
   return IIDC::successful;
}

IIDC::CameraError CamDriver::deinit () throw () {
  IIDC::CameraError res = IIDC::successful;
  if (startDone || dmaInitDone)
    res = stopTransmission ();
  if (res==IIDC::successful)
    initDone=false;
  return res;
}


IIDC::CameraError CamDriver::startTransmission() throw () {
  if (startDone) stopTransmission ();
  
   // evaluate which possible mode is similar to the desired mode
   availableFormat (mode_dc1394, framerate_dc1394, cameraFormat, colorCoding, framerate, imageWidth, imageHeight, offsetX, offsetY);
   
   // Check if enough bus bandwidth is available ..
   unsigned int bandwidth_needed = estimateBandwidth(colorCoding, framerate, imageWidth, imageHeight);
   if(bandwidth_used + bandwidth_needed > 400*1024*1024){
      ERROR_OUT << "Could not start camera capture due to limited bandwidth." << std::endl;
      ERROR_OUT << "A total bandwidth of approx. "<< (bandwidth_used+bandwidth_needed)/(1024.0*1024.0);
      ERROR_OUT << " Mbit/sec would be required." << std::endl;
      return IIDC::crowdedBus;
   }

   //Initialize the cameras capture behavior
   if (cameraFormat==IIDC::Format7) {
     // Bildausschnitt, Farbcodierung und Framerate einstellen
     unsigned int maxw=640, maxh=480;
     if (dc1394_query_format7_max_image_size(handle, camera.node, mode_dc1394, &maxw, &maxh)!=DC1394_SUCCESS) {
       ERROR_OUT << "Error querying maximal image size" << std::endl;
       return IIDC::dmaInitFailed;
     }
     if (dc1394_set_format7_image_size(handle, camera.node, mode_dc1394, imageWidth, imageHeight)!=DC1394_SUCCESS) {
       ERROR_OUT << "Error setting format7 image size" << std::endl;
       return IIDC::dmaInitFailed;
     }
     if (dc1394_set_format7_image_position(handle, camera.node, mode_dc1394, offsetX, offsetY)!=DC1394_SUCCESS) {
       ERROR_OUT << "Error setting format7 image position" << std::endl;
       return IIDC::dmaInitFailed;
     }
     if (dc1394_set_format7_color_coding_id(handle, camera.node, mode_dc1394, libdc_colorCoding_mapping[colorCoding])!=DC1394_SUCCESS) {
       ERROR_OUT << "Error setting format7 color coding" << std::endl;
       return IIDC::dmaInitFailed;
     }
     double bpp_opt = imageWidth*imageHeight*250e-6*framerate;  // laut Kameramanual: bytes_per_packet = width*height*byte_depth*fps*125us
     unsigned int bpp_ceil = static_cast<unsigned int>(bpp_opt);
     if (bpp_ceil<bpp_opt) bpp_ceil++;  // aufrunden
     if (bpp_ceil%4!=0)
       bpp_ceil = bpp_ceil-(bpp_ceil%4)+4;  // durch 4 teilbare Zahl
     if (bpp_ceil>4096)
       bpp_ceil = 4096;
     if (dc1394_set_format7_byte_per_packet (handle, camera.node, mode_dc1394, bpp_ceil) != DC1394_SUCCESS)
     {
       ERROR_OUT << "Error initializing frame rate format 7" << std::endl;
       return IIDC::dmaInitFailed;
     }
       
     // Kamera soll im Format7-Modus laufen
    if (dc1394_dma_setup_format7_capture(handle,
        camera.node,
        channel,
        mode_dc1394,
        SPEED_400,
        QUERY_FROM_CAMERA,       // Paketgroesse in Bytes
        static_cast<unsigned int>(QUERY_FROM_CAMERA),       // linker Rand (static_cast wegen Inkonsistenz in libdc)
        static_cast<unsigned int>(QUERY_FROM_CAMERA),       // oberer Rand (dito)
        static_cast<unsigned int>(QUERY_FROM_CAMERA),       // Bildbreite (dito)
        static_cast<unsigned int>(QUERY_FROM_CAMERA),       // Bildhoehe (dito)
        5,           // Anzahl DMA-Puffer
        1,           // drop frame
        device_name.c_str(),
        &camera ) == DC1394_FAILURE)
    {
      ERROR_OUT << "Error initializing dma-capture for format 7 camera uid=" << getCameraUID() << std::endl;
      return IIDC::dmaInitFailed;
     }
     // Ende Format7-Einstellungen
   } else {
    // Kamera soll im Format0-Modus laufen
    if (dc1394_dma_setup_capture (handle,
                                  camera.node, channel, FORMAT_VGA_NONCOMPRESSED, mode_dc1394,
                                  SPEED_400, framerate_dc1394,
                                  5,		// Buffers  //5 buffers is ok 2 will lead to hangups
                                  1,		// drop frames
                                  device_name.c_str(), &camera) == DC1394_FAILURE)
    {
        ERROR_OUT << "Error initializing dma-capture camera uid=" << getCameraUID() << std::endl;
        return IIDC::dmaInitFailed;
    }
  }

  dmaInitDone=true;
       
   // Start iso transmission
   if (dc1394_start_iso_transmission(handle, camera.node) != DC1394_SUCCESS)
   {
      ERROR_OUT << "Start of iso transmission failed" << std::endl;
      return IIDC::startTransmissionFailed;
    }

   // Update instance fields...
   startDone=true;
   bandwidth_used += bandwidth_needed;
   bandwidth_cam = bandwidth_needed;
   firstFrame = true;
   failure=0;

   return IIDC::successful;
}


IIDC::CameraError CamDriver::stopTransmission () throw () {
  dc1394_stop_iso_transmission (handle, camera.node);
  if (startDone && !firstFrame)
    dc1394_dma_done_with_buffer(&camera);
  startDone = false;
  bandwidth_used -= bandwidth_cam;
  bandwidth_cam=0;
  int success2 = dc1394_dma_unlisten (handle, &camera);
  int success3 = dc1394_dma_release_camera (handle, &camera);
  if (success2==DC1394_SUCCESS && success3==DC1394_SUCCESS) {
    dmaInitDone=false;
  }
  return ((success3==DC1394_SUCCESS) && (success2==DC1394_SUCCESS) ? IIDC::successful : IIDC::unknownReason);
}


CamDriver::~CamDriver() throw ()
{
  if (startDone || dmaInitDone)
    stopTransmission ();
  if (initDone)
    deinit ();
  dc1394_destroy_handle (handle);
}


ImageBuffer CamDriver::getImage () throw (ImageFailed)
{
  IIDC::CameraError restartError=IIDC::successful;
  if (!initDone || !dmaInitDone || !startDone)
    restartError = restartCamera();
  if (restartError!=IIDC::successful) deinit();
    
  int returnval = DC1394_FAILURE;          // hold the return value of dc calls

  if (initDone && dmaInitDone && startDone) {
    if (!firstFrame)
       dc1394_dma_done_with_buffer(&camera);

    // To get an image, the dma_sinlge_capture call is used in the blocking
    // (waiting) mode. This is more efficient than using the polling method,
    // but has a general problem: It's not possible to specify a timeout,
    // after that the call should return in case of a problem. Therefore,
    // when occuring a hardware problem, the capture call would never return.
    //
    // To handle this problem, we register a SIGALRM signal handler, that
    // guarantees us a return after 90 ms (suggested by Martin Lauer).

    signal(SIGALRM, &sighandler_nop);      // register empty signal handler
    itimerval t1, t2;
    t1.it_interval.tv_sec  = 0;
    t1.it_interval.tv_usec = 0;
    t1.it_value.tv_sec     = 0;
    t1.it_value.tv_usec    = 900000;       // set 90 ms timer
    setitimer (ITIMER_REAL, &t1, &t2) ;    // to return from blocking call

    if(isBlocking)
       returnval = dc1394_dma_single_capture(&camera) ;  //  blocking call
    else
       returnval = dc1394_dma_single_capture_poll(&camera); // non-blocking

    t1.it_interval.tv_sec  =
    t1.it_interval.tv_usec =
    t1.it_value.tv_sec     =
    t1.it_value.tv_usec    = 0;            // set to zero to stop timer
    setitimer (ITIMER_REAL, &t2, NULL);    // stop timer

  }

  // If no new frame was ready (in polling mode): firstFrame = true
  //   -> dont release buffer next time (would cause error)
  // And check if the reason was a bus reset (eg camera has been unplugged).
  // This is neccessary, because neither a bus reset nor a DC_FAILURE is
  // produced, if a camera in polling mode is unplugged.
  if(returnval == DC1394_NO_FRAME) {
     dummy_read(&handle);    // check if a bus reset is the reason for the
                             // unsuccessful capture.
     firstFrame = true;
     throw ImageFailed();  // tell calling function that no image has been obtained
  } else
    firstFrame = false;

  // in most cases the dma_single_capture fails, the connection to the camera
  // was physically lost. to repair those cases, the method simply waits for
  // 100ms before it tries to do a dummy read on the bus. If a device
  // has been replugged in the meantime, such a read would cause a
  // bus-reset-event to be emitted.
  //
  // On such an event, our own registered reset handler is called. The reset
  // handler does everything necessary to restart the replugged device.

  if ( returnval != DC1394_SUCCESS ) {     // capture didn't succeed
    firstFrame = true;
    if (restartError!=IIDC::cameraNotFound && restartError!=IIDC::crowdedBus)
      failure++;
    if (failure>=3) {
      failure=0;
      INFO_OUT << "Try bus reset." << endl;
      raw1394_reset_bus (handle);
    }
    
    usleep(100000);                        // sleep 100 ms
    dummy_read(&handle);                   // try to detect a bus reset
    throw ImageFailed();
    
  } else {                                 // DC1394_SUCCESS

    //    Time imagetime (camera.filltime);
    Time imagetime;  // camera.filltime ist ein kaputter Zeitstempel! (Martin Lauer)
    imagetime.add_msec (-camera_delay);
  
    ++frameCount;
    doSoftwareBalance();                   // call balancing routine

    // determine output format
    int outputFormat;
    double bytePP = bytePerPixel(colorCoding);    // byte per pixel
    switch (colorCoding) {
      case IIDC::Mono8: outputFormat = ImageBuffer::FORMAT_MONO; break;
      case IIDC::Mono16: outputFormat = ImageBuffer::FORMAT_MONO16; break;
      case IIDC::RGB8: outputFormat = ImageBuffer::FORMAT_RGB; break;
      case IIDC::RGB16: outputFormat = ImageBuffer::FORMAT_RGB; break;
      case IIDC::YUV411: outputFormat = ImageBuffer::FORMAT_YUV411; break;
      case IIDC::YUV422: outputFormat = ImageBuffer::FORMAT_YUV422; break;
      case IIDC::YUV444: outputFormat = ImageBuffer::FORMAT_YUV444; break;
      default: outputFormat = ImageBuffer::FORMAT_MONO; break;
    }

    // wrap an ImageBuffer around libdc1394_control's memory that holds the image
    return ImageBuffer(camera.frame_width,
		       camera.frame_height,
		       outputFormat,
		       (unsigned char *) camera.capture_buffer,
		       (int)(camera.frame_width*camera.frame_height*bytePP), 
                       imagetime);
  }
}

// ------------ possible mode and bandwidth ---------------------------------

unsigned int CamDriver::estimateBandwidth(IIDC::ColorCoding cc, float framerate, unsigned int iw, unsigned int ih) throw () {
  // slange: das unterschaetzt die benoetigte Bandbreite bei weitem, da nicht die gesamte
  //         Kapazitaet eines Frames fuer den isochronen Datentransfer vorgesehen ist und
  //         noch einiges Protokollgedoens hinzukommt. Hier vielleicht lieber die Tabelle
  //         aus der DFW-Dokumentation oder aus der IIDC-Spezifikation nachimplementieren...
  return static_cast<unsigned int>(framerate*iw*ih*((bytePerPixel(cc)+1)));
}

float CamDriver::bytePerPixel (IIDC::ColorCoding cc) throw () {
  switch (cc) {
    case IIDC::Mono8: return 1.0;
    case IIDC::Mono16: return 2.0;
    case IIDC::RGB8: return 3.0;
    case IIDC::RGB16: return 6.0;
    case IIDC::YUV411: return 1.5;
    case IIDC::YUV422: return 2.0;
    case IIDC::YUV444: return 3.0;
    default: return 1111111.0;
  }
}

void CamDriver::availableFormat (int& mode_dc1394, int& framerate_dc1394, IIDC::CameraFormat cf, IIDC::ColorCoding cc, float& fps, unsigned int& iw, unsigned int& ih, unsigned int&offsetX, unsigned int& offsetY) throw () {
  switch (cf) {
    case IIDC::Format7:
    {
      unsigned int maxw=100000, maxh=100000;
      dc1394_query_format7_max_image_size (handle, camera.node, MODE_FORMAT7_0, &maxw, &maxh);
      if (iw>maxw) iw=maxw;
      if (ih>maxh) ih=maxh;
      mode_dc1394 = MODE_FORMAT7_0;
      break;
    }
    case IIDC::Format0:
    default:
      if (cc==IIDC::YUV444) {
        iw=160;
        ih=120;
        mode_dc1394 = MODE_160x120_YUV444;
      } else if (cc==IIDC::YUV422) {
        if (iw>480 && ih>360) {
          iw=640;
          ih=480;
          mode_dc1394 = MODE_640x480_YUV422;
        } else {
          iw=320;
          ih=240;
          mode_dc1394 = MODE_320x240_YUV422;
        }
      } else if (cc==IIDC::YUV411) {
        iw=640;
        ih=480;
        mode_dc1394 = MODE_640x480_YUV411;
      } else if (cc==IIDC::Mono8) {
        iw=640;
        ih=480;
        mode_dc1394 = MODE_640x480_MONO;
      } else if (cc==IIDC::Mono16) {
        iw=640;
        ih=480;
        mode_dc1394 = MODE_640x480_MONO16;
      } else if (cc==IIDC::RGB8) {
        iw=640;
        ih=480;
        mode_dc1394 = MODE_640x480_RGB;
      } else {
        iw=ih=0;
        mode_dc1394 = MODE_640x480_MONO;  // default
      }
  }
  if (fps<=1.875) { framerate_dc1394 = FRAMERATE_1_875; fps=1.875; }
  else if (fps<=3.75) { framerate_dc1394 = FRAMERATE_3_75; fps=3.75; }
  else if (fps<=7.5) { framerate_dc1394 = FRAMERATE_7_5; fps=7.5; }
  else if (fps<=15.0) { framerate_dc1394 = FRAMERATE_15; fps=15.0; }
  else if (fps<=30.0) { framerate_dc1394 = FRAMERATE_30; fps=30.0; }
  else { framerate_dc1394 = FRAMERATE_60; fps=60.0; }
}



// ------------------------------- general camera queries -----------------------------------

int CamDriver::getWidth() const throw () {
  return imageWidth;
}

int CamDriver::getHeight() const throw () {
  return imageHeight;
}

int CamDriver::getOffsetX() const throw() {
  return offsetX;
}

int CamDriver::getOffsetY() const throw() {
  return offsetY;
}

std::string CamDriver::getCameraType () throw () {
  dc1394_camerainfo ci;
  dc1394_get_camera_info (handle, camera.node, &ci);
  std::string ret;
  ret = std::string(ci.vendor)+std::string(" ")+std::string(ci.model);
  return ret;
}

std::string CamDriver::getCameraMode () throw () {
  stringstream io;
  io << (cameraFormat==IIDC::Format0 ? "Format0" : "Format7") << ' ' << imageWidth << 'x' <<imageHeight << ' ';
  io << (colorCoding==IIDC::Mono8 ? "Mono8" : (colorCoding==IIDC::Mono16 ? "Mono16" : (colorCoding==IIDC::RGB8 ? "RGB8" : (colorCoding==IIDC::RGB16 ? "RGB16" : (colorCoding==IIDC::YUV411 ? "YUV411" : (colorCoding==IIDC::YUV422 ? "YUV422" : (colorCoding==IIDC::YUV444 ? "YUV444" : "unknownColorCoding"))))))) << ' ';
  io << framerate << "fps\n";
  std::string res;
  getline (io, res);
  return res;
}

bool CamDriver::revealCameraMode (IIDC::CameraFormat& tf, IIDC::ColorCoding& cc, float& fps, unsigned int& iw, unsigned int& ih, unsigned int& ox, unsigned int& oy, const std::string& arg) throw () {
  ox=oy=0;  // default
  vector<string> parts;
  split_string (parts, arg);
  bool format_found=false;
  bool size_found=false;
  bool offset_found=false;
  bool color_found=false;
  bool fps_found=false;

  for (unsigned int pp=0; pp<parts.size(); pp++) {  
    if (parts[pp].substr(0,6)=="Format") {
      // Kameraformat:
      if (parts[pp]=="Format0") tf = IIDC::Format0;
      else if (parts[pp]=="Format7") tf = IIDC::Format7;
      else {
        cerr << "cannot interpret camera format '" << parts[pp] << "'\n";
        return false;
      }
      format_found=true;
    } else if (parts[pp].find("x")!=string::npos) {
      // Bildgroesse:
      unsigned int p = parts[pp].find ("x",0);
      if (p>=parts[pp].length()) return false;
      if (!string2uint (iw, parts[pp].substr(0,p))) return false;
      if (!string2uint (ih, parts[pp].substr(p+1,parts[pp].size()))) return false;
      size_found=true;
    } else if (parts[pp].find("off")!=string::npos) {      
      // Offset:
      unsigned int p = parts[pp].find("off",0);
      if (!string2uint (ox, parts[pp].substr(0,p))) return false;
      if (!string2uint (oy, parts[pp].substr(p+3,parts[pp].size()))) return false;
      offset_found=true;
    } else if (parts[pp].find("fps")!=string::npos) {      
      unsigned int p = parts[pp].find ("fps",0);
      if (p>=parts[pp].length()) return false;
      if (!string2float (fps, parts[pp].substr (0,p))) return false;
      fps_found=true;
    } else {
      // Farbcodierung:
      if (parts[pp]=="YUV411") cc = IIDC::YUV411;
      else if (parts[pp]=="YUV422") cc = IIDC::YUV422;
      else if (parts[pp]=="YUV444") cc = IIDC::YUV444;
      else if (parts[pp]=="RGB8") cc = IIDC::RGB8;
      else if (parts[pp]=="RGB16") cc = IIDC::RGB16;
      else if (parts[pp]=="Mono8") cc = IIDC::Mono8;
      else if (parts[pp]=="Mono16") cc = IIDC::Mono16;
      else {
        cerr << "cannot interpret color coding '" << parts[pp] << "'\n";
        return false;
      }
      color_found=true;
    }
  }
  return (format_found && size_found && color_found && fps_found);
}

std::string CamDriver::getCameraUID () const throw () {
  return uid;
}

int CamDriver::getDelay () const throw () {
  return camera_delay;
}

std::string CamDriver::getCameraUID (raw1394handle_t handle, nodeid_t node) throw () {
  dc1394_camerainfo info;
  dc1394_get_camera_info(handle, node, &info);
  char id_buf[20];
  quadlet_t value[2];
  value[0]= info.euid_64 & 0xffffffff;
  value[1]= (info.euid_64 >>32) & 0xffffffff;
  sprintf(id_buf, "0x%08x%08x", value[1], value[0]);
  return string (id_buf);
}



// ------------------------- CameraFeatures ----------------------------

bool CamDriver::setWhiteBalance (unsigned int u, unsigned int v) throw () {
  unsigned int i;
  for (i=0; i<feature_set.size(); i++) {
    if (feature_set[i].feature==IIDC::whiteBalance) {
      feature_set[i].value = u;
      feature_set[i].second_value = v;
      break;
    }
  }
  if (i==feature_set.size()) {
    DCFeature nf;
    nf.feature=IIDC::whiteBalance;
    nf.mode=IIDC::featureMan;
    nf.value=u;
    nf.second_value=v;
    feature_set.push_back (nf);
  }
  return (dc1394_set_white_balance (handle, camera.node, u, v)==DC1394_SUCCESS);
}

bool CamDriver::getWhiteBalance (unsigned int& u, unsigned int& v) throw () {
  return (dc1394_get_white_balance (handle, camera.node, &u, &v)==DC1394_SUCCESS);
}

unsigned int CamDriver::getFeatureValue (IIDC::CameraFeature f) throw () {
  unsigned int val;
  dc1394_get_feature_value (handle, camera.node, libdc_feature_mapping[f], &val);
  return val;
}

bool CamDriver::setFeatureValue (IIDC::CameraFeature f, unsigned int val) throw () {
  unsigned int i;
  for (i=0; i<feature_set.size(); i++) {
    if (feature_set[i].feature==f) {
      feature_set[i].value = val;
      break;
    }
  }
  if (i==feature_set.size()) {
    DCFeature nf;
    nf.feature=f;
    nf.mode=IIDC::featureMan;
    nf.value=val;
    feature_set.push_back (nf);
  }
  Time timer; timer.update();
  stringstream str;
  bool ret = dc1394_set_feature_value (handle, camera.node, libdc_feature_mapping[f], val)==DC1394_SUCCESS;
  str << "dc1394_set_feature_value: " << (timer.elapsed_usec()/1000.,2) << "usecs";
  JMESSAGE(str.str().c_str());
  return ret;
}

IIDC::CameraFeatureMode CamDriver::getFeatureMode (IIDC::CameraFeature f) throw () {
  dc1394bool_t val;
  dc1394_is_feature_on (handle, camera.node, libdc_feature_mapping[f], &val);
  if (!val) return IIDC::featureOff;
  dc1394_is_feature_auto (handle, camera.node, libdc_feature_mapping[f], &val);
  if (val) return IIDC::featureAuto;
  dc1394_is_feature_present (handle, camera.node, libdc_feature_mapping[f], &val);
  if (val) return IIDC::featureMan;
  return IIDC::featureUnavailable;
}
  
bool CamDriver::setFeatureMode (IIDC::CameraFeature f, IIDC::CameraFeatureMode m) throw () {
  unsigned int i;
  for (i=0; i<feature_set.size(); i++) {
    if (feature_set[i].feature==f) {
      feature_set[i].mode = m;
      break;
    }
  }
  if (i==feature_set.size()) {
    DCFeature nf;
    nf.feature=f;
    nf.mode=m;
    feature_set.push_back (nf);
  }
  bool success=true;
  if (m & IIDC::featureOff) 
    success = (dc1394_feature_on_off (handle, camera.node, libdc_feature_mapping[f], 0)==DC1394_SUCCESS);
  else {
    success &= (dc1394_feature_on_off (handle, camera.node, libdc_feature_mapping[f], 1)==DC1394_SUCCESS);
    if (m & IIDC::featureAuto)
      success &= (dc1394_auto_on_off (handle, camera.node, libdc_feature_mapping[f], 1)==DC1394_SUCCESS);
    else
      success &= (dc1394_auto_on_off (handle, camera.node, libdc_feature_mapping[f], 0)==DC1394_SUCCESS);
  }
  return success;
}
  
unsigned int CamDriver::availableFeatureModes (IIDC::CameraFeature f) throw () {
  dc1394bool_t present, manual, automatic, off;
  dc1394_is_feature_present (handle, camera.node, libdc_feature_mapping[f], &present);
  dc1394_has_manual_mode (handle, camera.node, libdc_feature_mapping[f], &manual);
  dc1394_has_auto_mode (handle, camera.node, libdc_feature_mapping[f], &automatic);
  dc1394_can_turn_on_off (handle, camera.node, libdc_feature_mapping[f], &off);
  unsigned res = 0;
  if (!present) return IIDC::featureUnavailable;
  if (off) res |= IIDC::featureOff;
  if (manual) res |= IIDC::featureMan;
  if (automatic) res |= IIDC::featureAuto;
  return res;
}

bool CamDriver::getFeatureMinMaxValue (unsigned int& minv, unsigned int& maxv, IIDC::CameraFeature f) throw () {
  bool success = true;
  success &= (dc1394_get_min_value(handle, camera.node, libdc_feature_mapping[f], &minv)==DC1394_SUCCESS);
  success &= (dc1394_get_max_value(handle, camera.node, libdc_feature_mapping[f], &maxv)==DC1394_SUCCESS);
  return success;
}

void CamDriver::setSavedCameraFeatures() throw () {
  for (unsigned int i=0; i<feature_set.size(); i++) {
    usleep (delay_iidc_commands);  // ein Delay von mehreren Millisekunden
    setFeatureMode (feature_set[i].feature, feature_set[i].mode);
    if (feature_set[i].mode == IIDC::featureMan) {
      if (feature_set[i].feature == IIDC::whiteBalance) {
        setWhiteBalance (feature_set[i].value, feature_set[i].second_value);
      } else {
        setFeatureValue (feature_set[i].feature, feature_set[i].value);
      }
    }
  }
}


// ---------------- Software Exposure/White Balance ----------------------

//
// at the moment soft exposure does not work properly for all cameras!
//
void CamDriver::doSoftwareBalance()
{
  if (! doSoftWhiteBalance && ! doSoftExposure) {
    return;
  }
  if (frameCount % 3 != 0) {
    return;
  }

  stringstream str;
  Time timer; timer.update();
  unsigned int oldShutter = 0;
  unsigned int newshutter = 0;
  unsigned int oldGain = 0;
  unsigned int newgain = 0;

  if (doSoftExposure) {
    oldShutter = getFeatureValue(IIDC::shutter);
    newshutter = oldShutter;
    oldGain = getFeatureValue(IIDC::gain);
    newgain = oldGain;
  }

  unsigned int oldU, oldV;
  if (!getWhiteBalance(oldU, oldV)) {
    ERROR_OUT << "Weissbalance konnte nicht gelesen werden." << endl;  // TODO: Loggen
    return ;
  }
  unsigned int u=oldU, v=oldV;
  str << "doSoftwareBalance: " << (timer.elapsed_usec() / 1000.) << " ";

  int iSize = (int) (imageWidth * imageHeight * bytePerPixel(colorCoding));
   
  double average  = 127.;
  double averageU = 127.;
  double averageV = 127.;
   
  switch(colorCoding) {
  case IIDC::YUV422:
  case IIDC::YUV411:
  case IIDC::YUV444: 
    {
      int start=0;
      int step=balanceStep * 12;
      int offsetU=1;
      int offsetV=2;
      double bytePerPixel = 3;
      switch (colorCoding) {
      case IIDC::YUV444: start = 0; step = balanceStep * 12; offsetU =  1; offsetV = 2; bytePerPixel = 3; break; 
      case IIDC::YUV422: start = 1; step = balanceStep *  8; offsetU = -1; offsetV = 1; bytePerPixel = 2; break;
      case IIDC::YUV411: start = 1; step = balanceStep *  6; offsetU = -1; offsetV = 2; bytePerPixel = 1.5; break;
      default: break;
      }
      if (doSoftWhiteBalance) {
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
            uV += ((unsigned char*)camera.capture_buffer)[pos+offsetU];
            vV += ((unsigned char*)camera.capture_buffer)[pos+offsetV];
            ++count;
          }
        }
        averageU = uV / (double)count;
        averageV = vV / (double)count;
      }
      if (doSoftExposure) {
	long value = 0; int count = 0;
	for (int c = start; c < iSize; c+=step) {
	  value += ((unsigned char*)camera.capture_buffer)[c];
	  ++count;
	}
	average  = value / (double)count;
      }
    }  
    break;
  default: break;
  }
  double uTmp = oldU * (1. - (averageU-127.) / 255);
  double vTmp = oldV * (1. - (averageV-127.) / 255);
  u = (unsigned int) (uTmp < oldU ? floor(uTmp) : ceil(uTmp));
  v = (unsigned int) (vTmp < oldV ? floor(vTmp) : ceil(vTmp));
 
  double normDelta = (shutterMax-shutterMin)/20;
  double error = (static_cast<double>(average)-static_cast<double>(software_exposure));
  if (doSoftExposure && abs(error)>=3) {
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
      newshutter = oldShutter+dShutter;
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
  u = u < uvMin ? uvMin : u > uvMax ? uvMax : u;
  v = v < uvMin ? uvMin : v > uvMax ? uvMax : v;
  if (doSoftWhiteBalance && (oldU != u || oldV != v)) {
    setWhiteBalance(u,v);
  }
  if (doSoftExposure && oldShutter != newshutter) {
    newshutter = newshutter < shutterMin ? shutterMin : 
                                     newshutter > shutterMax ? shutterMax : newshutter;
    setFeatureValue(IIDC::shutter, newshutter);
  }
  if (doSoftExposure && adjustGain && oldGain != newgain) {
    newgain = newgain < gainMin ? gainMin : newgain > gainMax ? gainMax : newgain;
    setFeatureValue(IIDC::gain, newgain);
  }
  str << (timer.elapsed_usec() / 1000.);
  JMESSAGE(str.str().c_str());
}

void CamDriver::toggleSoftwareExposure(bool on) throw()
{ doSoftExposure = on; }

bool CamDriver::isSoftwareExposure() const throw()
{ return doSoftExposure; }

void CamDriver::toggleSoftwareWhiteBalance(bool on) throw()
{ doSoftWhiteBalance = on; }

bool CamDriver::isSoftwareWhiteBalance() const throw()
{ return doSoftWhiteBalance; }

void 
CamDriver::initSoftBalance(bool doSoftExposure, bool doSoftWhiteBalance, 
                      bool adjustGain, int balanceStep,
                      unsigned char exposure,
                      int shutterLogic,
                      unsigned int shutterMin, 
                      unsigned int shutterMax,
                      unsigned int gainMin, 
                      unsigned int gainMax,
                      unsigned int uvMin, 
                      unsigned int uvMax)
{
  this->doSoftExposure = doSoftExposure;
  this->doSoftWhiteBalance = doSoftWhiteBalance;
  this->adjustGain = adjustGain;
  this->balanceStep = balanceStep;  // TODO: remove, no meanding for autobalance
  this->shutterLogic = shutterLogic;
  this->shutterMin = shutterMin;
  this->shutterMax = shutterMax;
  this->gainMin = gainMin;
  this->gainMax = gainMax;
  this->uvMin = uvMin;
  this->uvMax = uvMax;
  this->software_exposure = exposure;

  if (shutterMin >= shutterMax) {
    getFeatureMinMaxValue(this->shutterMin, this->shutterMax, IIDC::shutter);
  }
  if (gainMin >= gainMax) {
    getFeatureMinMaxValue(this->gainMin, this->gainMax, IIDC::gain);
  }
  
  if (uvMin >= uvMax) {
    getFeatureMinMaxValue(this->uvMin, this->uvMax, IIDC::whiteBalance);
  }

  autoExposureError=0;
  autoExposureDelta=(shutterMax-shutterMin)/20;
}

// ------------------ load/save memory channel ------------------------------
bool CamDriver::loadSettings(int channel) throw ()
{
  return (dc1394_memory_load (handle, camera.node, channel)==DC1394_SUCCESS);
}

bool CamDriver::saveSettings(int ch) throw ()
{
  bool success = (dc1394_set_memory_save_ch (handle, camera.node, ch)==DC1394_SUCCESS);
  if (success)
    return (dc1394_memory_save (handle, camera.node)==DC1394_SUCCESS);
  else
    return false;
}

unsigned int CamDriver::numChannel() throw() {
  dc1394_miscinfo mi;
  dc1394_get_camera_misc_info(handle, camera.node, &mi);
  return mi.mem_channel_number;
}

bool CamDriver::checkCamera(std::ostream& logout) throw () {
  logout << "camera uid=" << uid << '\n';
  if (!initDone) {
    logout << "camera not initialized/fatal\n";
    return false;
  } else {
    logout << "camera initialized/okay\n";
  }
  if (!dmaInitDone) {
    logout << "camera DMA not started/fatal\n";
    return false;
  } else {
    logout << "camera DMA started/okay\n";
  }    
  if (!startDone) {
    logout << "camera not yet started/fatal\n";
    return false;
  } else {
    logout << "camera started\n";
  }
  if (firstFrame) {
    logout << "camera did not capture first frame, cannot check image/failure counter=" << failure << '\n';
    return false;
  }
  if ((colorCoding==IIDC::YUV411) || (colorCoding==IIDC::YUV422) || (colorCoding==IIDC::YUV444)) {
    // pruefen, ob tatsaechlich ein Farbbild vorliegt oder nur ein Schwarz/Weiss-Bild
    // RGB wird momentan nicht unterstuetzt
    double bytePerPixel = 3;
    int offsetU=1;
    int offsetV=2;
    int tuplelen=3;
    int num_tuples = imageWidth*imageHeight;
    switch (colorCoding) {
      case IIDC::YUV444: offsetU =  1; offsetV = 2; tuplelen=3; bytePerPixel = 3; break;
      case IIDC::YUV422: offsetU =  0; offsetV = 2; tuplelen=4; bytePerPixel = 2; num_tuples/=2; break;
      case IIDC::YUV411: offsetU =  0; offsetV = 3; tuplelen=6; bytePerPixel = 1.5; num_tuples/=4; break;
      default: break;
    }
    unsigned int num_grey_pixel=0;
    for (int ti=0; ti<num_tuples; ti++) {
      if ((((unsigned char*)camera.capture_buffer)[ti*tuplelen+offsetU]==128 && 
           ((unsigned char*)camera.capture_buffer)[ti*tuplelen+offsetV]==128 ) || (
           ((unsigned char*)camera.capture_buffer)[ti*tuplelen+offsetU]==127 && 
           ((unsigned char*)camera.capture_buffer)[ti*tuplelen+offsetV]==127 )
         ) {
        num_grey_pixel++;
      }
    }
    int colorrate = ((num_tuples-num_grey_pixel)*100)/num_tuples;
    if (colorrate>20) {
      logout << colorrate << "% colored pixels found/okay\n";
    } else {
      logout << colorrate << "% colored pixels found although camera started in color mode/fatal\n";
      return false;
    }
  }
  // Features pruefen
  bool features_okay=true;
  for (unsigned int fn=0; fn<feature_set.size(); fn++) {
    DCFeature fval = feature_set[fn];
    usleep (delay_iidc_commands);  // ein Delay von mehreren Millisekunden
    IIDC::CameraFeatureMode mode = getFeatureMode (feature_set[fn].feature);
    if (feature_set[fn].mode != mode) {
      logout << "discrepancy in feature mode " << featurename_mapping [feature_set[fn].feature] << ": " << featuremodename_mapping[mode] << " found, " << featuremodename_mapping[feature_set[fn].mode] << " expected/fatal\n";
      features_okay=false;
    }
    if (feature_set[fn].mode==IIDC::featureMan) {
      usleep (delay_iidc_commands);  // ein Delay von mehreren Millisekunden
      if (feature_set[fn].feature==IIDC::whiteBalance) {
        unsigned int v1, v2;
        getWhiteBalance (v1, v2);
        if (feature_set[fn].value!=v1 || feature_set[fn].second_value!=v2) {
          logout << "discrepancy in feature value " << featurename_mapping [feature_set[fn].feature] << ": " << v1 << '/' << v2 << " found, " << feature_set[fn].value << '/' << feature_set[fn].second_value << " expected/fatal\n";
          features_okay=false;
        }
      } else {
        unsigned int value = getFeatureValue (feature_set[fn].feature);
        if (feature_set[fn].value != value) {
          logout << "discrepancy in feature value " << featurename_mapping [feature_set[fn].feature] << ": " << value << " found, " << feature_set[fn].value << " expected/fatal\n";
          features_okay=false;
        }
      }
    }
    logout << "feature " << featurename_mapping [feature_set[fn].feature] << " okay\n";
  }
  return features_okay;
}      
