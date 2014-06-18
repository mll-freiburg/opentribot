#include <string>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include "compass.h"


using namespace std;

#define GYRO_GAIN_SCALE 64.  // standardwert laut dokumentation, soll laut kalibrierungsdokument bei unserem Gyro aber 10000 sein... glaub ich nicht!


/** returns a fd */
int compass_init(char* devicename)
{
  int fd;
  struct termios newtio;

  // INITIALISIEREN /////////////////////////////////////
  fd = open(devicename, O_RDWR | O_NOCTTY | O_NONBLOCK ); 
  
  if (fd < 0) {
    return fd;
  }
   
  bzero(&newtio, sizeof(newtio));

  newtio.c_cflag = B38400 | CS8 | CREAD;
  newtio.c_iflag = 0;
  newtio.c_oflag = 0;
  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;
  
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd,TCSANOW,&newtio);

  return fd;
}

int compass_talk(int fd, 
		 const char* cmd, int cmd_size, 
		 char* result, int result_size)
{
  if (write(fd, cmd, cmd_size) != cmd_size) {
    perror("Error trying to write");
    return -1;
  }
  for (int c=0; c < result_size; c++) {
    while (read(fd, result+c, 1) != 1) ;  // Zeichen einzeln lesen
  }
  return 0;
}  

int compass_getEuler(int fd, double* roll, double* pitch, double* yaw)
{
  char cmd = 0x0E; // get stabilized euler angles
  char result[11];
  
  if (compass_talk(fd, &cmd, 1, result, 11) != 0) {
    return -1;
  }
  *roll  = (((int)result[1] << 8) | (unsigned char) result[2]) * (360./65536.);
  *pitch = (((int)result[3] << 8) | (unsigned char) result[4]) * (360./65536.);
  *yaw   = (((int)result[5] << 8) | (unsigned char) result[6]) * (360./65536.);

  return 0;
}  

/** returns gyro stabilized vectors.
 *  mag:   1 "Earth field unit"
 *  accel: 1 G (9.81 m/sec^2)
 *  gyro:  rad/s
 */
int compass_getVectors(int fd, struct 
		       compass_vector *mag,
		       compass_vector *accel,
		       compass_vector *gyro)
{
  char cmd = 0x02; // get stabilized vectors
  char result[23];
  
  if (compass_talk(fd, &cmd, 1, result, 23) != 0) {
    return -1;
  }

  mag->x = (((int)result[1] << 8) | (unsigned char) result[2]) / 8192.;
  mag->y = (((int)result[3] << 8) | (unsigned char) result[4]) / 8192.;
  mag->z = (((int)result[5] << 8) | (unsigned char) result[6]) / 8192.;


  accel->x = (((int)result[ 7] << 8) | (unsigned char) result[ 8]) / 8192.;
  accel->y = (((int)result[ 9] << 8) | (unsigned char) result[10]) / 8192.;
  accel->z = (((int)result[11] << 8) | (unsigned char) result[12]) / 8192.;

  double gyroScale = GYRO_GAIN_SCALE * 8192. * 0.0065536;

  gyro->x = (((int)result[13] << 8) | (unsigned char) result[14]) / gyroScale;
  gyro->y = (((int)result[15] << 8) | (unsigned char) result[16]) / gyroScale;
  gyro->z = (((int)result[17] << 8) | (unsigned char) result[18]) / gyroScale;

  return 0;
}  

/** returns raw gyro vectors.
 *  mag:   1 "Earth field unit"
 *  accel: 1 G (9.81 m/sec^2)
 *  gyro:  rad/s
 */
int compass_getInstantaneousVectors(int fd, struct
    compass_vector *mag,
    compass_vector *accel,
    compass_vector *gyro)
{
  char cmd = 0x03; // get nicht-stabilisierte vectors
  char result[23];
  
  if (compass_talk(fd, &cmd, 1, result, 23) != 0) {
    return -1;
  }

  mag->x = (((int)result[1] << 8) | (unsigned char) result[2]) / 8192.;
  mag->y = (((int)result[3] << 8) | (unsigned char) result[4]) / 8192.;
  mag->z = (((int)result[5] << 8) | (unsigned char) result[6]) / 8192.;


  accel->x = (((int)result[ 7] << 8) | (unsigned char) result[ 8]) / 8192.;
  accel->y = (((int)result[ 9] << 8) | (unsigned char) result[10]) / 8192.;
  accel->z = (((int)result[11] << 8) | (unsigned char) result[12]) / 8192.;

  double gyroScale = GYRO_GAIN_SCALE * 8192. * 0.0065536;

  gyro->x = (((int)result[13] << 8) | (unsigned char) result[14]) / gyroScale;
  gyro->y = (((int)result[15] << 8) | (unsigned char) result[16]) / gyroScale;
  gyro->z = (((int)result[17] << 8) | (unsigned char) result[18]) / gyroScale;

  return 0;
}
