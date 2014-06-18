#include "serialimu.h"

#include <cstdio>
#include <unistd.h>
#include <sys/time.h>

GyroKalman::GyroKalman(){
	Q_angle=0.00002f;
	Q_gyro=0.1;//0.1f;
	R_angle=0.3f;
	P_00=0;
	P_01=0;
	P_10=0;
	P_11=0;
}



void GyroKalman::predict(double dotAngle, double dt) {

  x_angle += dt * (dotAngle - x_bias);
  P_00 += -1 * dt * (P_10 + P_01) + dt*dt * P_11 + Q_angle;
  P_01 += -1 * dt * P_11;
  P_10 += -1 * dt * P_11;
  P_11 += Q_gyro;
}


double GyroKalman::update(double angle_m) {
  const double y = angle_m - x_angle;
  const double S = P_00 + R_angle;
  const double K_0 = P_00 / S;
  const double K_1 = P_10 / S;

  x_angle += K_0 * y;
  x_bias  += K_1 * y;
  
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;
  
  return x_angle;
}



SerialImu::SerialImu(){
calibratecount=0;
yaw0=0;
pitch0=0;
roll0=0;

}

int SerialImu::calibrate_zeroes(int y,int p ,int r){

	if (y>0&&p>0&&r>0){
	printf ("\nCalibrating... YPR %i %i %i ",y,p,r);
	yaw0+=y;
	pitch0+=p;
	roll0+=r;
	calibratecount++;
	printf ("... Sum.. YPR %i %i %i ",yaw0,pitch0,roll0);

	if (calibratecount==100){
        yaw0=yaw0/100;
	pitch0=pitch0/100;
	roll0=roll0/100;
	printf ("\\Gyro Calibration done zeros: %f %f %f   ",yaw0,pitch0,roll0);
           }
	


	}
	else {
	printf("Got wrong values during calibration , waiting... " );

	return 0;
	}

}
	
	

  






void ImuFilter::processAccelerometers(double ax,double ay,double az){


  //The real purpose of this method is to convert the accelometer values into usable angle values.  
  //Knuckles904 pointed me to this app note: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
  //That paper gives the math for implementing a tilt sensor using 3-axis accelerometers.  Roll(X) and pitch(Y) are directly
  //applicable.  Yaw(Z) is not since there is no 'tilt' with respect to the earth.  
  
  //Once the accelerometer values have been biased to zero (by subtracting 511 above), then they should fall in a range from
  //-512 to +511.

  

}




int
SerialImu::ser_init ()		//default st_bit = 2 (header)
{

  serial_filedescriptor =
    open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_SYNC);
  if (serial_filedescriptor < 0)
    {

      cout << "Cannot open Serial Port" << endl;
      return -1;
    }

  tcgetattr (serial_filedescriptor, &oldtio);	/* save current serial port settings */

  bzero (&newtio, sizeof (newtio));	/* clear struct for new port settings */


  newtio.c_cflag = B115200 |  CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR | ICRNL;
  newtio.c_oflag = 0;
  newtio.c_lflag = (ICANON | ECHOE | ISIG); // CANONICAL MODE
  tcflush (serial_filedescriptor, TCIOFLUSH);
  tcsetattr (serial_filedescriptor, TCSANOW, &newtio);

  return 0;




}




int
SerialImu::ser_deinit ()
{


  if (tcsetattr (serial_filedescriptor, TCSANOW, &oldtio) == -1)
    {
      perror ("tcsetattr");
      return -1;
    }
  close (serial_filedescriptor);

  return 0;

}


int
SerialImu::ser_send (char *send)
{
  long start_time = get_current_time ();	//Startup-Zeit

  if (write (serial_filedescriptor, send, strlen (send)) < 0)
    {
      ERROR_OUT << "Can't send a command via Serial line\n" << endl;
      return -1;
    }
  else
    {
//  cout<< "send command took "<<get_current_time()-start_time<<"ms"<<endl;

    }
  return 0;
}


int
SerialImu::update_and_filter ()
{

}


int
SerialImu::ser_receive (char *receive, int recv_len)
{
  int n, retval, rsize;
  fd_set rfds;
n=0;
  struct timeval tv;

  long start_time = get_current_time ();	//Startup-Zeit

  FD_ZERO (&rfds);
  FD_SET (serial_filedescriptor, &rfds);
  tv.tv_sec = 1;
  tv.tv_usec = 0;
//  std::cout<<"vor select:"<<get_current_time()-start_time;
  retval = select (serial_filedescriptor + 1, &rfds, NULL, NULL, &tv);
  //retval = select (FD_SETSIZE, &rfds, NULL, NULL, &tv);

//  std::cout<<"nach select:"<<get_current_time()-start_time;
  if (FD_ISSET (serial_filedescriptor, &rfds))
    {

      n = read (serial_filedescriptor, receive, 1000);

  //    cout << n  <<"gelesen"<<endl;
  //    std::cout<<"nach read:"<<get_current_time()-start_time<<std::endl;
      return n;
    }
  return n;

}


int
SerialImu::send (float *xmp, float *ymp, float *pmp)
{
   int res;
char tmc_message[100];
  sprintf (tmc_message, "SV " );
  //cout <<"DAS SENDE ICH "<<tmc_message<<endl;
  // Und ans Tmxc Schicken
  if ((res = ser_send (tmc_message)) < 0)
    {
      ERROR_OUT << "TMC Send Error" << endl;
    }


  return res;
}





int
SerialImu::imu_get_values ()
{
  int res = 0;
  char *tmc_answer;
  tmc_answer = new char[BUFSIZE];

  if ((res = ser_receive (tmc_answer, BUFSIZE - 1)) < 0)
    {
      cout << "serial communication error: %d\n" << endl;
    }
  if( res <10) return 0;

  char * pch,val;
  pch = strtok (tmc_answer," ");
  bz=0;bc=0;fy=0;fp=0;fr=0;
  while (pch != NULL)
  {
    if (strcmp(pch,"ax")==0){
	pch=strtok(NULL," ");
	if(pch!=NULL)
		ax=atoi(pch);	}
    if (strcmp(pch,"ay")==0){
	pch=strtok(NULL," ");if(pch!=NULL)ay=atoi(pch); }
    if (strcmp(pch,"az")==0){
	pch=strtok(NULL," ");if(pch!=NULL)az=atoi(pch); }
    if (strcmp(pch,"y")==0){
	pch=strtok(NULL," ");if(pch!=NULL)y=atoi(pch);	}
    if (strcmp(pch,"p")==0){
	pch=strtok(NULL," ");if(pch!=NULL)r=atoi(pch);	}
    if (strcmp(pch,"r")==0){
	pch=strtok(NULL," ");if(pch!=NULL)p=atoi(pch);	}
    if (strcmp(pch,"jy")==0){
	pch=strtok(NULL," ");if(pch!=NULL)jy=atoi(pch);	}
    if (strcmp(pch,"jx")==0){
	pch=strtok(NULL," ");if(pch!=NULL)jx=atoi(pch);	}
    if (strcmp(pch,"LR")==0){
	pch=strtok(NULL," ");if(pch!=NULL){dt_ms=atoi(pch);dt_s=dt_ms*1.0f/1000;} }	
    if (strcmp(pch,"FP")==0){fr=true;}
    if (strcmp(pch,"FY")==0){fy=true;}
    if (strcmp(pch,"FR")==0){fp=true;}
    if (strcmp(pch,"zb")==0)bc=1;
    if (strcmp(pch,"cb")==0)bz=1;
    if (pch!=NULL) pch = strtok (NULL, " ");

  }

  if (calibratecount<100){
	calibrate_zeroes(y,p,r);
  acc_x=ax;
  acc_y=ay;
  acc_z=az;



}

  else { 
   // Start integrating stuff
   yaw   =-((y-yaw0)*1.0f/20);
   pitch =((p-pitch0)*1.0f/20);
   roll  =((r-roll0)*1.0f/20);
   if (fp)pitch*=4;
   if (fy)yaw*=4;
   if (fr)roll*=4;

  acc_x=ax;
  acc_y=ay;
  acc_z=az;
  
// acc x accy accz is vector!
//double ref_x=0,ref_y=0;ref_z=1;



cout <<"\ny p r "<<yaw <<" "<< pitch <<" "<< roll;
if (fp||fr||fy)cout <<"Fast !!!";
cout <<endl;

   /*acc_x=acc_x*0.998f+0.002f*(ax-511);
   acc_y=acc_y*0.998f+0.002f*(ay-511);
   acc_z=acc_z*0.998f+0.002f*(az-511);
*/
   
  double x = filter.convert2G(267, 690,495, acc_x);
  double y = filter.convert2G(299, 720,511, acc_y);
  double z = filter.convert2G(340,780,570, acc_z);
  filter.g_x=x;
  filter.g_y=y;
  filter.g_z=z;
 


  //compute values that are used in multiple places

//  filter.accelAngleX = Rad2Deg*atan(x / sqrt(y*y + z*z));
//  filter.accelAngleY = Rad2Deg*atan(y / sqrt(x*x + z*z));

double xcut=x;
if (xcut>1.0f)xcut=1.0f;
if (xcut<-1.0f)xcut=-1.0f;
//filter.accelAngleX = Rad2Deg*asin(xcut);
  filter.accelAngleX = Rad2Deg*atan2(x,z);
  filter.accelAngleY = Rad2Deg*atan2(y,z);


//cout << "Roll:"<filter.accelAngleX<<" Pitch "<<filter.accelAngleY<<endl;  

  filter.theta       = Rad2Deg*atan(sqrt(x*x+y*y) / z);



//

/*V3d direction(0,y,z);
V3d up(x,0,z);
V3d right;
right=up.crossprod(direction);
double m3[9];
m3[0]=right.x;m3[1]=right.y;m3[2]=right.z;
m3[3]=right.x;m3[4]=right.y;m3[5]=right.z;
m3[6]=right.x;m3[7]=right.y;m3[8]=right.z;
Quaternion quata;
quata.r=sqrt(1+right.x+up.y+direction.z)/2.0f;
double wscale=quata.r*4;
quata.x=(direction.y-up.z)/wscale;
quata.y=(right.z-direction.x)/wscale;
quata.z=(up.x-right.y)/wscale;
quata.normalize();
*/


Quaternion accel;
/*
 * cout << "AccelAngleY"<<filter.accelAngleY*Deg2Rad<<endl;
cout << "AccelAngleX"<<filter.accelAngleX*Deg2Rad<<endl;
*/




  /*if (filter.theta<0){

  filter.accelAngleX = Rad2Deg*(M_PI-atan(x / sqrt(y*y + z*z)));
  filter.accelAngleY = Rad2Deg*(M_PI-atan(y / sqrt(x*x + z*z)));



}





*/





   //filter.rollData.predict(-roll * Deg2Rad,dt_s);
   //filter.pitchData.predict(-pitch * Deg2Rad,dt_s);
   //filter.kroll = Rad2Deg*filter.rollData.update(filter.accelAngleX*Deg2Rad);
   //filter.kpitch = Rad2Deg*filter.pitchData.update(filter.accelAngleY*Deg2Rad);
   //filter.kpitch = Rad2Deg*filter.pitchRK.computeRK4(-pitch*Deg2Rad*dt_s);
   //filter.kroll = Rad2Deg*filter.rollRK.computeRK4(-roll*Deg2Rad*dt_s);
    double scale_gyro=1;//90*1.0f/195;;
    
     long us=get_current_usec();
     
/*
     cout<<endl << us<<" absolute Usecs passed=" <<us-last_usec<<" dt  "<<dt_s<<endl;
     last_usec=us;
*/
/*    filter.kpitch+= pitch*dt_s*scale_gyro;//
    filter.kroll+= roll*dt_s*scale_gyro;//
    filter.rkyaw += yaw*dt_s*scale_gyro;// Rad2Deg*filter.yawRK.computeRK4(yaw*Deg2Rad*dt_s);
*/ 


// Orientation quaternion update:
    Quaternion qgyro;

//    roll=0;pitch=0;yaw=0;

     qgyro.set(0,pitch*dt_s*Deg2Rad,yaw*dt_s*Deg2Rad,roll*dt_s*Deg2Rad);

    double gp,gy,gr;
     Quaternion delta;
     delta=(filter.gyro_int*qgyro)*0.5f;
     filter.gyro_int=filter.gyro_int+delta;
     filter.gyro_int.toEuler(&gp,&gy,&gr);
	

    accel.fromEuler(-filter.accelAngleY*Deg2Rad,gy,-filter.accelAngleX*Deg2Rad);//accel.set(1,-x,-z,y);
    accel=accel.normalize();


    //qgyro.fromEuler(Deg2Rad*roll*dt_s*scale_gyro,Deg2Rad*pitch*dt_s*scale_gyro,Deg2Rad*yaw*dt_s*scale_gyro);
//     filter.gyro_int=filter.orient+delta;
  /*  Quaternion deltar,deltap,deltay;
    deltar.set(roll*dt_s*scale_gyro*Deg2Rad,1,0,0);
    deltap.set(pitch*dt_s*scale_gyro*Deg2Rad,0,1,0); 
    deltay.set(yaw*dt_s*scale_gyro*Deg2Rad,0,0,1); 
    */
/*
    filter.orient=filter.orient*deltar;
    filter.orient=filter.orient*deltay;
    filter.orient=filter.orient*deltap;
*/
//    filter.orient=filter.orient*qgyro;

//    filter.orient.nlerp(accel,0.011001f);
//    filter.orient=accel;
    filter.orient=filter.gyro_int;
    filter.orient=filter.orient.normalize();


    
    cout <<" gbeforerot"<< filter.g_x<<" y "<<filter.g_y<<" z "<<filter.g_z<<endl;


    Quaternion temp,rota;
temp.set(0,filter.g_x,filter.g_z,filter.g_y);
   
    rota=filter.orient.get_conjugate()*temp*filter.orient;
filter.think_x=rota.x;
filter.think_y=rota.y;
filter.think_z=rota.z;

 cout <<" rota x"<< rota.x<<" y "<<rota.y<<" z "<<rota.z<<endl;
    V3d int_3d(rota.x,rota.y,rota.z);
    V3d acc_3d(0,-1,0);
    cout <<" rot2 x"<< rota.x<<" y "<<rota.y<<" z "<<rota.z<<endl;
    V3d axis=acc_3d.crossprod(int_3d);
   int_3d=int_3d.normalize();
   acc_3d=acc_3d.normalize();
   // cout <<" ax x"<< axis.x<<" y "<<axis.y<<" z "<<axis.z<<endl;
    double ang=acos(int_3d.dotprod(acc_3d));
    cout << "The angle is "<< ang/M_PI*180<<endl;
   Quaternion rollback;
   rollback.fromAxisAngle(ang,axis);

   Quaternion rotated;
   rotated=filter.orient*rollback;

    filter.gyro_int.normalize();
   filter.gyro_int.nlerp(rotated,0.01001f);

    filter.orient=filter.gyro_int;

/*
    filter.g_x=rota.x;
   filter.g_y=rota.y;
   filter.g_z=rota.z;
*/

    filter.orient.toMatrix4x4(filter.mat4x4); 
//printf("QUAT %8.03f %8.03f %8.03f %8.03f",filter.orient.r,filter.orient.x,filter.orient.y,filter.orient.z);


 //filter.rkyaw = Rad2Deg*filter.yawRK.computeRK4(yaw*Deg2Rad*dt_s);
//cout<<endl<< "dt_s=<<"<<dt_s<<endl;
//   filter.kpitch=filter.kpitch*0.99+filter.accelAngleY*0.01;
 //  filter.kroll=filter.kroll*0.99+filter.accelAngleX*0.01;


  }

  return res;
}



//cout <<"nach scanff"<<endl;

  //Motor v umrechnen
//cout<<"nach ser_receive"<<endl;








//----------------------------------------------------------------------------
#ifdef TEST

int
main ()
{

  SerialImu imu;

  imu.ser_init ();
  int i = 0;

int count=0;
  while (true)
    {
    if(imu.imu_get_values()>5){
	count++;
 // Raw Values
//    printf("COUNT:%i y%i p%i r%i ax%i ay%i az%i jx%i jy%i bz%i bc%i\r\n",count,imu.y,imu.p,imu.r,imu.ax,imu.ay,imu.az,imu.jx,imu.jy,imu.bz,imu.bc);
// real values
printf("\n y%6.02f p%6.02f r%6.02f ax%6.02f ay%6.02f ta%6.02f kr%6.02f kp%6.02f rky%6.02f",imu.yaw,imu.pitch,imu.roll,imu.filter.accelAngleX,imu.filter.accelAngleY,imu.filter.theta*Rad2Deg,imu.filter.kroll,imu.filter.kpitch,imu.filter.rkyaw
)  ;

   i++;
	}
    }


  return 0;
}


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
#endif

long
SerialImu::get_current_time ()
{
  timeval tval;
  if (gettimeofday (&tval, NULL))
    std::cerr << "\n something wrong with time mesurement";
  return tval.tv_sec * 1000 + tval.tv_usec / 1000;

}
long SerialImu::get_current_usec ()
{
  timeval tval;
  if (gettimeofday (&tval, NULL))
    std::cerr << "\n something wrong with time mesurement";
  return tval.tv_usec;
}
