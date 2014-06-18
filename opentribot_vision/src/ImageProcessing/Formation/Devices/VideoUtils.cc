#include "VideoUtils.h"

#define DL "\n(" << __LINE__ << "): "
#define DEB(XXX) std::cout << DL << XXX
//#define DEB(XXX)
#define ERR(XXX) std::cout << DL << XXX

VideoUtils::VideoUtils()
{
}

VideoUtils::~VideoUtils()
{
}


long VideoUtils::get_current_ms_time() {
  timeval tval;
  static long s_time_at_start= 0;
  if (gettimeofday(&tval,NULL))
    ERR("something wrong with time mesurement");
  
  if ( 0 == s_time_at_start )
    s_time_at_start= tval.tv_sec;
  
  return (tval.tv_sec - s_time_at_start) * 1000 + tval.tv_usec / 1000;
}

bool VideoUtils::save_yuv422(char const* fname, unsigned char const* buf, int x, int y) {
  FILE *file;
  if ((file = fopen (fname, "w+")) == NULL ) {
    ERR("cannot open file");
    return false;
  }
  fwrite ((unsigned char *) buf, x * y * 2, sizeof(unsigned char), file);
  fclose (file); 
}

bool VideoUtils::save_yuv420p(char const* fname, unsigned char const* buf, int x, int y) {
  FILE *file;
  char fname_tmp[strlen(fname)+2];
  sprintf (fname_tmp, "%s.Y", fname);
  if ((file = fopen (fname_tmp, "w+")) == NULL) {
    ERR("could not open file " << fname_tmp);
    return false;
  }
  fwrite ((unsigned char *) buf, x * y, 1, file);
  fclose (file); 

  sprintf (fname_tmp, "%s.U", fname);
  if ((file = fopen (fname_tmp, "w+")) == NULL) {
    ERR("could not open file " << fname_tmp);
    return false;
  }
  fwrite ((unsigned char *) buf+ x*y, (x * y)/4, 1, file);
  fclose (file); 

  sprintf (fname_tmp, "%s.V", fname);
  if ((file = fopen (fname_tmp, "w+")) == NULL) {
    ERR("could not open file " << fname_tmp);
    return false;
  }
  fwrite ((unsigned char *) buf+ x*y+(x*y)/4, (x * y)/4, 1, file);
  fclose (file); 

  return true;
} 

void VideoUtils::component(unsigned char const* src, unsigned char* dest, int width, int height) {
  unsigned char * odd_line;
  for (int i=0; i< height/2; i++) {
    odd_line= dest+width*2;
    for (int j=0; j< width/2; j++) {
      *dest= *src;
      *odd_line= *src;
      src++;
      dest+=4;
      odd_line+=4;
    }
    dest= odd_line; //skip the next line
  }
}

void VideoUtils::yuv420p_to_yuv422(unsigned char const* src, unsigned char* dest, int x,int y) {
  /* -> uyvy */
  int hsize= x*y;
  int qqsize= (x*y)/4;
  unsigned char * ddd=dest+1;
  /* y component */
  ddd= dest+1;
  for (int i=0; i<hsize; i++) {
    *ddd= *src;
    src++;
    ddd+=2;
  }
  /* u component */
  component(src,dest,x,y);
  /* v component */
  component(src+qqsize,dest+2,x,y);
}

bool VideoUtils::save_yuv420p_as_yuv422(char const* fname,unsigned char const* buf, int x, int y) {
  unsigned char dum[x*y*2];
  yuv420p_to_yuv422(buf,dum,x,y);
  save_yuv422(fname,dum,x,y);
} 
