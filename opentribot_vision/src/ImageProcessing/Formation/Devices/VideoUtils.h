#ifndef VIDEOUTILS_H_
#define VIDEOUTILS_H_

#include <string.h> 
#include <sys/time.h>
//#include <linux/videodev.h>
#include <sys/mman.h> 
#include <sys/ioctl.h> 
#include <fcntl.h>
#include <errno.h> 
#include <iomanip>
#include <iostream>

class VideoUtils
{
public:
	VideoUtils();
	virtual ~VideoUtils();

static long get_current_ms_time();

static bool save_yuv422(char const* fname, unsigned char const* buf, int x, int y);

static bool save_yuv420p(char const* fname, unsigned char const* buf, int x, int y);

static void component(unsigned char const* src, unsigned char* dest, int width, int height);

static void yuv420p_to_yuv422(unsigned char const* src, unsigned char* dest, int x,int y);

static bool save_yuv420p_as_yuv422(char const* fname,unsigned char const* buf, int x, int y);


};

#endif /*VIDEOUTILS_H_*/
