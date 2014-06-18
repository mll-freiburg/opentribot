#ifndef _IMU_H_
#define _IMU_H_


#include <sys/time.h>

#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <math.h>

#define BUFSIZE 1000

#ifndef ERROR_STREAM
#define ERROR_STREAM cerr
#endif

#ifndef ERROR_OUT
#define ERROR_OUT ERROR_STREAM << "ERR:" << __FILE__ << " l:" << __LINE__<<" "
#ifndef WARNING_STREAM
#define WARNING_STREAM cerr
#endif

#ifndef WARNING_OUT
#define WARNING_OUT WARNING_STREAM << "WARN:" << __FILE__ <<" l:" << __LINE__<<" "
#endif
#endif

#include "usbimu.h"

using namespace std;


struct V3d{
double x,y,z;
V3d(){x=0;y=0;z=0;}
V3d(double x_ , double y_ ,double z_){x=x_;y=y_;z=z_;}
V3d operator+(V3d v){return V3d(x+v.x,y+v.y,z+v.z);}
V3d operator-(V3d v){return V3d(x-v.x,y-v.y,z-v.z);}
double norm(){return (x*x+z*z+y*y);}
double length(){return sqrt(x*x+y*y+z*z);}
V3d normalize(){double l=length();return V3d(x/l,y/l,z/l);}
V3d crossprod(V3d v){return V3d(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x);}
double dotprod(V3d v){return x*v.x+y*v.y+z*v.z;}
}


;



struct Quaternion{


double r;
double x;
double y;
double z;

Quaternion(double _r,double _x,double _y,double _z){r=_r;x=_x;y=_y;z=_z; }
Quaternion(){r=0;x=0;y=0;z=1; }
void set(double _r,double _x,double _y,double _z){r=_r;x=_x;y=_y;z=_z; }

double norm(){
    return (r*r+x*x+y*y+z*z);
}
Quaternion get_inverse(){
return get_conjugate()*(1/norm());

}
void fromAxisAngle(double angle, V3d axis)
{
V3d ax=axis.normalize();
double sin_a=sin(angle/2);
double cos_a=cos(angle/2);
x=sin_a*ax.x;
y=sin_a*ax.y;
z=sin_a*ax.z;
r=cos_a;

}
void fromEuler(double roll, double pitch, double yaw)
{
float cr, cp, cy, sr, sp, sy, cpcy, spsy;

// compute all trigonometric values used to compute the quaternion
 cr = cos(roll/2);
 cp = cos(pitch/2);
 cy = cos(yaw/2);
//
 sr = sin(roll/2);
 sp = sin(pitch/2);
 sy = sin(yaw/2);
//
 cpcy = cp * cy;
 spsy = sp * sy;
//
// // combine values to generate the vector and scalar for the quaternion
 r = cr * cpcy + sr * spsy;
 x = sr * cpcy - cr * spsy;
 y = cr * sp * cy + sr * cp * sy;
 z = cr * cp * sy - sr * sp * cy;
 normalize(); 
}

void toEuler(double * roll,   double* pitch,  double* yaw)
{

*roll=atan2(2*(r*x+y*z),1-2*(x*x+y*y));
*pitch=asin(2*(r*y-z*x));
*yaw=atan2(2*(r*z+x*y),1-2*(y*y+z*z));

}


double length(){
    return (sqrt(r*r+x*x+y*y+z*z));
}
Quaternion get_conjugate(){
return Quaternion(r,-x,-y,-z);
}
Quaternion  normalize(){
 
     double l=length();
     r/=l;
     x/=l;
     y/=l;
     z/=l;
return Quaternion(r/l,x/l,y/l,z/l);

}
void toMatrix4x4(double*mat){
double xx=x*x;
double xy=x*y;
double xz=x*z;
double xw=x*r;
double yy=y*y;
double yz=y*z;
double yw=y*r;
double zz=z*z;
double zw=z*r;
mat[0]= 1-2*(yy+zz);
mat[1]=   2*(xy-zw);
mat[2]=   2*(xz+yw);
mat[4]=   2*(xy+zw);
mat[5]=1- 2*(xx+zz);
mat[6]=   2*(yz-xw);
mat[8]=   2*(xz-yw);
mat[9]=   2*(yz+xw);
mat[10]=1-2*(xx+yy);
mat[3]=mat[7]=mat[11]=mat[12]=mat[13]=mat[14]=0;
mat[15]=1;


}

void nlerp( const Quaternion to, float t)
	{
	float oneminust = 1.0f - t;
	r=r*oneminust+to.r*t;
	x=x*oneminust+to.x*t;
	y=y*oneminust+to.y*t;
	z=z*oneminust+to.z*t;
	normalize();
	}



void multiply_with(Quaternion q){
r=r*q.r- x*q.x - y*q.y - z*q.z;
x=r*q.x- x*q.r - y*q.z - z*q.y;
y=r*q.y- x*q.z - y*q.r - z*q.x;
z=r*q.z- x*q.y - y*q.x - z*q.r;
}

V3d  operator*(V3d v){


};


Quaternion operator*(Quaternion q){
Quaternion ret;
ret.r=r*q.r- x*q.x - y*q.y - z*q.z;
ret.x=r*q.x + x*q.r + y*q.z - z*q.y;
ret.y=r*q.y +y*q.r  +z*q.x  - x*q.z ;
ret.z=r*q.z+ x*q.y - y*q.x + z*q.r;
return ret;
}
Quaternion operator*(double s){
Quaternion ret;
ret.r=r*s;
ret.x=x*s;
ret.y=y*s;
ret.z=z*s;
return ret;
}
Quaternion operator+(Quaternion q){
Quaternion ret;
ret.r=r+q.r;
ret.x=x+q.x;
ret.y=y+q.y;
ret.z=z+q.z;
return ret;
}




};

struct GyroKalman
{
 GyroKalman();  
/*
 * The kalman predict method.  See http://en.wikipedia.org/wiki/Kalman_filter#Predict
 *
 * kalman    the kalman data structure
 * dotAngle  Derivitive Of The (D O T) Angle.  This is the change in the angle from the gyro.  This is the value from the
 *           Wii MotionPlus, scaled to fast/slow.
 * dt        the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
 *
 * Note: Tom Pycke's ars.c code was the direct inspiration for this.  However, his implementation of this method was inconsistent
 *       with the matrix algebra that it came from.  So I went with the matrix algebra and tweaked his implementation here.
 */
void predict(double,double);  
 /*
 * The kalman update method.  See http://en.wikipedia.org/wiki/Kalman_filter#Update
 *
 * kalman    the kalman data structure
 * angle_m   the angle observed from the Wii Nunchuk accelerometer, in radians
 */
 double  update(double);  

/* These variables represent our state matrix x */
  double x_angle,
         x_bias;

  /* Our error covariance matrix */
  double P_00,
         P_01,
         P_10,
         P_11;

  /*
 *    * Q is a 2x2 matrix of the covariance. Because we
 *       * assume the gyro and accelerometer noise to be independent
 *          * of each other, the covariances on the / diagonal are 0.
 *             *
 *                * Covariance Q, the process noise, from the assumption
 *                   *    x = F x + B u + w
 *                      * with w having a normal distribution with covariance Q.
 *                         * (covariance = E[ (X - E[X])*(X - E[X])' ]
 *                            * We assume is linear with dt
 *                               */
  double Q_angle, Q_gyro;

  /*
 *    * Covariance R, our observation noise (from the accelerometer)
 *       * Also assumed to be linear with dt
 *          */
  double R_angle;



};


struct RungeKutta {
double val_i_3;
double val_i_2;
double val_i_1;
double previous;
RungeKutta(){
val_i_3=0;
val_i_2=0;
val_i_1=0;
previous=0;

}
double computeRK4(double val_i_0){
previous+=0.16667*(val_i_3+2*val_i_2+2*val_i_1+val_i_0);
val_i_3=val_i_2;
val_i_2=val_i_1;
val_i_1=val_i_0;
return previous;

}

};


static  int wmpSlowToDegreePerSec = 20;
static int wmpFastToDegreePerSec = wmpSlowToDegreePerSec/5;

static const double SecondsPerMillis = 0.0001;
static const double Deg2Rad=M_PI/180;
static const double Rad2Deg=180.0f/M_PI;

//WM+ variables
/*
 * R represents the measurement covariance noise.  In this case,
  * it is a 1x1 matrix that says that we expect 0.3 rad jitter
   * from the accelerometer.
    */
    static const float	R_angle	= .3; //.3 default

    /*
     * Q is a 2x2 matrix that represents the process covariance noise.
      * In this case, it indicates how much we trust the acceleromter
       * relative to the gyros.
        *
         * You should play with different values here as the effects are interesting.  Over prioritizing the
          * accelerometers results in fairly inaccurate results.
           */
           static const double  Q_angle = 0.002; //(Kalman)
           static const double  Q_gyro  = 0.1; //(Kalman)


struct ImuFilter{
/*
 * Nunchuk accelerometer value to radian conversion.   Normalizes the measured value
 * into the range specified by hi-lo.  Most of the time lo should be 0, and hi should be 1023.
 *
 * lo         the lowest possible reading from the Nunchuk
 * hi         the highest possible reading from the Nunchuk
 * measured   the actual measurement
 */
float convert2G(int lo, int hi,int mid, int measured) {
  float x = (hi - lo);
  return (float)((measured-mid)/x) ;
}


void processAccelerometers(double x,double y,double z);

// Kalman data structures for each rotational axis
struct GyroKalman rollData;
struct GyroKalman pitchData;
struct GyroKalman yawData;
struct RungeKutta yawRK;
struct RungeKutta pitchRK;
struct RungeKutta rollRK;

// raw 3 axis gyro readings
int readingsX;
int readingsY;
int readingsZ;		  

// 3 axis gs
double g_x;
double g_y;
double g_z;

double think_x;
double think_y;
double think_z;


char data[6]; //six data bytes

// normalized (scaled per WM+ mode) rotation values
int yaw ;
int pitch ;
int roll ;

  Quaternion orient;
  Quaternion gyro_int;

double mat4x4[16];

// WM+ state variables - if true, then in slow (high res) mode

bool debug ;

// Nunchuck variables

float accelAngleX;	//NunChuck X angle
float accelAngleY;	//NunChuck Y angle
float accelAngleZ;	//NunChuck Z angle
float theta;          //Angle of the z-axis to vertical (not the same as yaw)

// Internal rogram state variables
unsigned long lastread; //last system clock in millis
  double kroll;
  double kpitch;
  double kyaw;
  double rkyaw;


};



class Imu {

public:
// raw values
  
long last_usec;


int ax,ay,az;
  int y,p,r,jy,jx;
  int bc;
  int bz;
  int dt_ms; // Zeitdifferenz
  double dt_s; // Zeitdifferenz
//processed values
 // Fast-Flags
  bool fy;
  bool fp;
  bool fr;
// calibration zeroes
  double yaw0 ;
  double pitch0 ;
  double roll0 ;
  double acc_x;
  double acc_y;
  double acc_z;

  double yaw,roll,pitch;
  
  int calibratecount;

  Imu();


  ImuFilter filter;
  
  int ser_init ();
  int ser_deinit ();
  int ser_send(char *send);
  int ser_receive(char *receive, int recv_len);
  int send(float * xmp,float * ymp,float * pmp);
  int imu_get_values();
  int calibrate_zeroes(int y,int p,int r);

  int update_and_filter();
  long get_current_time();
  long get_current_usec();

  USBImu usbimu;


};

#endif



