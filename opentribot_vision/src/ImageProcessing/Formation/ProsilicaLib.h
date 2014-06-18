/* header files */
//#include <raw1394.h>
#include <libdc1394/dc1394_control.h>
//#include <libdc1394/dc1394_internal.h>

/* macro definitions */
#define CAM_WIDTH	640
#define CAM_HEIGHT	480

#define CAM_SUCCESS	0
#define CAM_ERROR	1

/* prototypes */
int init_camera(raw1394handle_t*, dc1394_cameracapture*, float);
int grab_image_YUV(unsigned char*, unsigned char*, unsigned char*, dc1394_cameracapture*);
int stop_camera(raw1394handle_t, dc1394_cameracapture*);
int set_camera_param(raw1394handle_t*, dc1394_cameracapture*,int* ,int);

int set_camera_param_long(raw1394handle_t*, dc1394_cameracapture* ,int* ,int );
int auto_calibrate_param(raw1394handle_t* , dc1394_cameracapture* , int );
int get_camera_param_long(raw1394handle_t*, dc1394_cameracapture* ,int* ,int );
