#ifndef QUICKCAMDEVICE_H_
#define QUICKCAMDEVICE_H_

#include <string.h> 
#include <sys/time.h>
#include <linux/videodev.h>
#include <sys/mman.h> 
#include <sys/ioctl.h> 
#include <fcntl.h>
#include <errno.h> 
#include <iomanip>
#include <iostream>

#include "pwc-ioctl.h"

#define MAX_BUFFERS 2

#define DL "\n(" << __LINE__ << "): "
//#define DEB(XXX) std::cout << DL << XXX
#define DEB(XXX)
#define ERR(XXX) std::cout << DL << XXX


class QuickcamDevice {

  bool dma_mode;
  bool select_mode;
  int size_x, size_y, depth;
  
  struct video_window grab_win;
  struct video_picture grab_pic;
  struct video_capability grab_cap;
  //struct video_mmap grab_buf;
  struct video_mbuf grab_mbuf;
  unsigned int grab_fd;

  unsigned char * buffer;  
  int buffer_size;
  int buffer_offsets[MAX_BUFFERS];
  int palette;
  int num_buffers;
  int HasFramerate;

 public:
  QuickcamDevice() {}

  bool open(char const* device);
  void close();
  bool capture(int buf_idx);
  bool sync(int buf_idx);

  unsigned char const* get_buffer(int idx) {
    //DEB("buffer_offsets[" << idx << "]= " << buffer_offsets[idx]);
    return buffer+ buffer_offsets[idx];
  }
 unsigned char * get_buffer_for_write(int idx) {
    //DEB("buffer_offsets[" << idx << "]= " << buffer_offsets[idx]);
    return buffer+ buffer_offsets[idx];
  }

  int get_size_x() { return size_x; }
  int get_size_y() { return size_y; }
  int get_num_buffers() { return num_buffers; }
  int get_fd() { return grab_fd; }
  void createDummyBuffers();
  int blend_buffers(int i,int j);
//debug
char const* show_palette(int palette);
};

#endif /*QuickcamDevice_H_*/
