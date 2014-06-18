#include "QuickcamDevice.h"

bool QuickcamDevice::open(char const* device) {
  size_x = 640;
  size_y = 480;
  depth = 3;
  HasFramerate = false;

  if ((grab_fd = ::open (device, O_RDWR)) < 0 ) {
    ERR("CANNOT open " << device);
    return false;
  }
  if ( ioctl(grab_fd, VIDIOCGWIN, &grab_win) < 0) {
    ERR("VIDIOCGWIN failure");
    return false;
  }
  size_x = grab_win.width;
  size_y = grab_win.height;
  DEB("VIDIOCGWIN ok:" << DL << "   video_window.size= " << size_x << ", " << size_y );

#if 1
    //#if defined(PWC_FPS_SHIFT)
    if ((grab_win.flags & PWC_FPS_FRMASK) >> PWC_FPS_SHIFT) {
      HasFramerate = true;
      //DEB("QuickcamDevice supports framerate setting [" << vwin.flags << "]");
    }
#endif

  if (ioctl(grab_fd, VIDIOCGPICT, &grab_pic) < 0) {
    ERR("VIDIOCGPICT failure, could not get picture parameters.");
    return false;
  }
  DEB("VIDIOCGPICT ok:");
  DEB("   video_picture.depth= " << grab_pic.depth);
  DEB("   video_picture.palette= " << show_palette(grab_pic.palette) );
  depth= grab_pic.depth;
  palette = grab_pic.palette; 

  if (ioctl(grab_fd, VIDIOCGCAP, &grab_cap) < 0) {
    ERR("VIDIOCGCAP failure, could not query capabilities!");
    return false;
  }
  grab_cap.name[31] = '\0';
  DEB("VIDIOCGCAP ok:");
  DEB("   video_capability.name= " << grab_cap.name);
  DEB("   video_capability.maxwidth= " << grab_cap.maxwidth );
  DEB("   video_capability.maxheight= " << grab_cap.maxheight );

#if 1
  //set frame size to max size
  DEB("<resize frame>");
  grab_win.width= grab_cap.maxwidth;
  grab_win.height= grab_cap.maxheight;
  if ( ioctl(grab_fd, VIDIOCSWIN, &grab_win) < 0) {
    ERR("VIDIOCGWIN failure");
    return false;
  }
  DEB("VIDIOCGWIN ok:" << DL << "   video_window.size= " << grab_win.width << ", " << grab_win.height );

  if ( ioctl(grab_fd, VIDIOCGWIN, &grab_win) < 0) {
    ERR("VIDIOCGWIN failure");
    return false;
  }

  size_x = grab_win.width;
  size_y = grab_win.height;
  DEB("VIDIOCGWIN ok:" << DL << "   video_window.size= " << size_x << ", " << size_y );
  DEB("</resize frame>");
#endif


  if ( strncmp(grab_cap.name,"Philips",7) == 0 )
    select_mode= true;
  else
    select_mode= false;

  if (ioctl(grab_fd, VIDIOCGMBUF, &grab_mbuf) == 0) {
    dma_mode= true;
    DEB("VIDIOCGMBUF ok:" << DL << "   using mmap" << DL << "   video_mbuf.size=" << grab_mbuf.size);
    DEB("   video_mbuf.frames= " << grab_mbuf.frames);
    buffer_size = grab_mbuf.size;
    num_buffers = grab_mbuf.frames;

    buffer = (unsigned char *)mmap(NULL, buffer_size, PROT_READ, MAP_SHARED, grab_fd, 0);
    if (buffer == (unsigned char *)-1) {
      ERR("CQuickcamDevice::Init(): mmap() failed (" << errno << "). Falling back to non-mmap()ed mode.");
      return false;
    }

    for (int i = 0; i < num_buffers; i++)
      buffer_offsets[i] = grab_mbuf.offsets[i];
  }
  else {
    dma_mode= false;
    DEB("VIDIOCGMBUF failure:" << DL << " no mmap possible, allocating own buffer.");
    if (num_buffers <= 0 || num_buffers > 4)
      num_buffers = 4; // In case the user didn't specify a buffer size, we make one.
    DEB("   num_buffers= " << num_buffers);
    buffer_size = num_buffers * size_x * size_y * 4;
    buffer = new unsigned char[buffer_size];
    for (int i = 0; i < num_buffers; i++)
      buffer_offsets[i] = i * size_x * size_y * 4;
  }
   
  // restore user settings 
  if (ioctl(grab_fd, VIDIOCPWCRUSER) == -1)
    ERR("couldn't load user settings");
  else
  	DEB("user settings loaded");
  
  DEB("video device opened");
  return true;
}

void QuickcamDevice::close(){
  DEB("closing video device");
  ::close(grab_fd);
}

bool QuickcamDevice::capture(int buf_idx) {
  if ( ! dma_mode ) {
    DEB("capture for non dma mode not implemented");
    return false;
  }

  struct video_mmap vm;
  vm.frame = buf_idx;
  vm.format = palette;
  vm.width = size_x;
  vm.height = size_y;

  DEB("buf_idx= " << buf_idx);

  if (ioctl(grab_fd, VIDIOCMCAPTURE, &vm) < 0) {
    DEB("VIDIOCMCAPTURE failure");
    perror("Error");
    //eval: errno 
    return false;
  }
  DEB("buffer " << buf_idx << " captured");
  return true;
}

bool QuickcamDevice::sync(int buf_idx) {
  int frame= buf_idx;
  if ( ioctl (grab_fd, VIDIOCSYNC, &frame) < 0 ) {
    DEB("VIDIOCSYNC failure");
    return false;
  }
  DEB("VIDIOCSYNC ok:" << DL << "   frame= " << frame << ", buf_idx= " << buf_idx);
  return true;
}

char const* QuickcamDevice::show_palette(int palette) {
  switch(palette) {
  case  VIDEO_PALETTE_GREY      : return "VIDEO_PALETTE_GREY";
  case  VIDEO_PALETTE_HI240   	: return "VIDEO_PALETTE_HI240";    
  case  VIDEO_PALETTE_RGB565  	: return "VIDEO_PALETTE_RGB565";   
  case  VIDEO_PALETTE_RGB24   	: return "VIDEO_PALETTE_RGB24";  
  case  VIDEO_PALETTE_RGB32   	: return "VIDEO_PALETTE_RGB32";    
  case  VIDEO_PALETTE_RGB555  	: return "VIDEO_PALETTE_RGB555";   
  case  VIDEO_PALETTE_YUV422  	: return "VIDEO_PALETTE_YUV422";   
  case  VIDEO_PALETTE_YUYV    	: return "VIDEO_PALETTE_YUYV";     
  case  VIDEO_PALETTE_UYVY    	: return "VIDEO_PALETTE_UYVY";     
  case  VIDEO_PALETTE_YUV420  	: return "VIDEO_PALETTE_YUV420";   
  case  VIDEO_PALETTE_YUV411  	: return "VIDEO_PALETTE_YUV411";   
  case  VIDEO_PALETTE_RAW     	: return "VIDEO_PALETTE_RAW";      
  case  VIDEO_PALETTE_YUV422P 	: return "VIDEO_PALETTE_YUV422P";
  case  VIDEO_PALETTE_YUV411P 	: return "VIDEO_PALETTE_YUV411P";  
  case  VIDEO_PALETTE_YUV420P 	: return "VIDEO_PALETTE_YUV420P";  
  case  VIDEO_PALETTE_YUV410P 	: return "VIDEO_PALETTE_YUV410P";  
    //  case  VIDEO_PALETTE_PLANAR  	: return "VIDEO_PALETTE_PLANAR";   
    //  case  VIDEO_PALETTE_COMPONENT	: return "VIDEO_PALETTE_COMPONENT";
  }
  return "UNKNOWN VIDEO PALETTE";
}


int QuickcamDevice::blend_buffers(int i,int j){
      unsigned char * tgt;
       unsigned char * src;
       src=buffer+buffer_offsets[i];
        tgt=buffer+buffer_offsets[j];
	int size=buffer_size/num_buffers;
         for (int v=0;v<size;v++){
		 if (tgt[v]==0)tgt[v]=src[v];
		 else tgt[v]=(src[v]+tgt[v])/2;
          }
	 std::cout<<"Blending buffer size" <<buffer_size<<std::endl;
           return 0;
 }
void QuickcamDevice::createDummyBuffers(){
  dma_mode= false;
    DEB("VIDIOCGMBUF failure:" << DL << "   no mmap possible, allocating own buffer.");
    if (num_buffers <= 0 || num_buffers > 4)
      num_buffers = 4; // In case the user didn't specify a buffer size, we make one.
    DEB("   num_buffers= " << num_buffers);
    buffer_size = num_buffers * size_x * size_y * 4;
    buffer = new unsigned char[buffer_size];
    for (int i = 0; i < num_buffers; i++)
      buffer_offsets[i] = i * size_x * size_y * 4;
return;
}
