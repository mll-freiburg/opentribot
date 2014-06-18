#ifndef _COMPASS_H_
#define _COMPASS_H_


struct compass_vector {
  double x, y, z;
};


/** returns a fd */
int compass_init(char* devicename);

int compass_talk(int fd, 
		 const char* cmd, int cmd_size, 
		 char* result, int result_size);

int compass_getEuler(int fd, double* roll, double* pitch, double* yaw);


/** returns gyro stabilized vectors.
 *  mag:   1 "Earth field unit"
 *  accel: 1 G (9.81 m/sec^2)
 *  gyro:  rad/s
 */
int compass_getVectors(int fd, struct 
		       compass_vector *mag,
		       compass_vector *accel,
		       compass_vector *gyro);

int compass_getInstantaneousVectors(int fd, struct
    compass_vector *mag,
    compass_vector *accel,
    compass_vector *gyro);


#endif
