#include "imu.h"

#include <cstdio>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <fstream>

GyroKalman::GyroKalman ()
{
    Q_angle = 0.00002f;
    Q_gyro = 0.1;			//0.1f;
    R_angle = 0.3f;
    P_00 = 0;
    P_01 = 0;
    P_10 = 0;
    P_11 = 0;
}



void
GyroKalman::predict (double dotAngle, double dt)
{

    x_angle += dt * (dotAngle - x_bias);
    P_00 += -1 * dt * (P_10 + P_01) + dt * dt * P_11 + Q_angle;
    P_01 += -1 * dt * P_11;
    P_10 += -1 * dt * P_11;
    P_11 += Q_gyro;
}


double
GyroKalman::update (double angle_m)
{
    const double y = angle_m - x_angle;
    const double S = P_00 + R_angle;
    const double K_0 = P_00 / S;
    const double K_1 = P_10 / S;

    x_angle += K_0 * y;
    x_bias += K_1 * y;

    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;

    return x_angle;
}



Imu::Imu ()
{
    calibrated=false;
    yaw0 = 0;
    pitch0 = 0;
    roll0 = 0;
    debug=true;
  #ifdef USE_USB_IMU
    usbdevice=new USBImu();
    usbdevice->init ();
    cout << "Using USB IMU INitialization"<<endl;
  #endif

}

int
Imu::gyro_calibrate_zero ()
{

    int calls=0;
    cout << "gyro calibration"<<endl;


    while (calls<100) {
        calls++;
        usbdevice->get (0);
        int y = usbdevice->gyro[0];
        int p = usbdevice->gyro[1];
        int r = usbdevice->gyro[2];

        //	printf(" y %i p %i r %i \n",y,p,r);
        if (y > 0 && p > 0 && r > 0)
        {
            printf ("\nCalibrating... YPR %i %i %i ", y, p, r);
            yaw0 += y;
            pitch0 += p;
            roll0 += r;
            //printf ("... Sum.. YPR %i %i %i ", yaw0, pitch0, roll0);




        }
        else
        {
            printf ("Got wrong values during calibration , waiting... ");

            return 0;
        }
    }

    yaw0 = yaw0 / 100;
    pitch0 = pitch0 / 100;
    roll0 = roll0 / 100;
  gyroaxis[0].zero=yaw0;
  gyroaxis[1].zero=pitch0;
  gyroaxis[2].zero=roll0;
    
    printf ("\\Gyro Calibration done zeros: %f %f %f   ", yaw0, pitch0,
            roll0);
    calibrated=true;;



    return 0;
}

int Imu::acc_calibrate_axis(calibratestate s,int axisnum,double value){
  
  Axis * a=&accaxis[axisnum];
  cout << "\nValue "<<value<<"    :"<<endl;;
  
  if (value>400 && value<600){
  cout << "must be zero"<< endl;
  a->zero=value;
  return 0;
  }
  
  
  cout << "must be extrema (1g)"<<endl;
  if (value > 500){
     a->scale=1;a->max=value;a->min=a->zero-(a->max-a->zero);
  }
  else {
     a->min=value;a->scale=-1;a->max=a->zero+(a->zero-a->min);
  }
if (s==FLAT){
	  cout << "Must be z"<<endl;
	  accmap[ACCZ]=axisnum;}
  
if (s==RIGHT){
	  cout << "Must be x"<<endl;
	  accmap[ACCX]=axisnum;}
if (s==FRONT){
	  cout << "Must be y"<<endl;
	  accmap[ACCY]=axisnum;}
return 0;
}
int Imu::gyro_calibrate_axis(calibratestate s,int axisnum,double value){
  
  Axis * a=&gyroaxis[axisnum];
  cout << "\nValue "<<value<<"    :"<<endl;;
  
  if (fabs(value)<60){
  cout << "must not have turned !"<< endl;
  
  return 0;
  }
  
  
  cout << "must be Largest value "<<endl;
  if (value > 0){
     a->scale=1;
  }
  else {
     a->scale=-1;
  }
  
if (s==RIGHT){
	  cout << "Must be ROLL"<<endl;
	  gyromap[ROLL]=axisnum;}
if (s==FRONT){
	  cout << "Must be< PITCH"<<endl;
	  gyromap[PITCH]=axisnum;}
  if (s==TURN){
	  cout << "Must be YAW"<<endl;
	  gyromap[YAW]=axisnum;
  }
  
  
  
  

return 0;
}


int
Imu::accelerometer_calibrate(calibratestate s)
{
    int calls=0;
    
	double a0 =0;
        double a1 =0;
        double a2 =0;
	double gint0 =0;
        double gint1 =0;
        double gint2 =0;

	
	
      double fac=0.99f;
	double avorher;
	double adiff=100;
      StopWatch sw;
	double d_t;
	sw.reset();
    while (adiff >1.0f) {
        
        
        calls++;
        usbdevice->get (0);
	avorher=a0+a1+a2;
	a0 = a0*fac+ usbdevice->acc[0]*(1-fac);
        a1 = a1*fac+ usbdevice->acc[1]*(1-fac);
        a2 = a2*fac+ usbdevice->acc[2]*(1-fac);
	
	d_t=1.0f*sw.get_usecs()/(1000*1000);
	sw.reset();
	gint0 += d_t*(usbdevice->gyro[0]-gyroaxis[0].zero)/20 ; //(/ 20   laut datenblatt gyro )
        gint1 += d_t*(usbdevice->gyro[1]-gyroaxis[1].zero)/20;
        gint2 += d_t*(usbdevice->gyro[2]-gyroaxis[2].zero)/20;
	
	adiff=adiff*0.8f+fabs(avorher-usbdevice->acc[0]-usbdevice->acc[1]-usbdevice->acc[2])*0.2f;
	//	cout << "adiff="<<adiff<<endl;
	printf ("\\ accdiff %f Gyro INTS:  %f %f %f  \n	 ", adiff,gint0,gint1,gint2);
	
    }
  
    printf ("\\Accelerometer Calibration done zeros: %f %f %f  \n	 ", a0, a1,
            a2);
    printf ("\\Gyro INTS:  %f %f %f  \n	 ", gint0,gint1,gint2);
	
	    
    acc_calibrate_axis(s,0,a0);
    acc_calibrate_axis(s,1,a1);
    acc_calibrate_axis(s,2,a2);
    

    gyro_calibrate_axis(s,0,gint0);
    gyro_calibrate_axis(s,1,gint1);
    gyro_calibrate_axis(s,2,gint2);


return 0;
}










int
Imu::deinit ()
{
    usbdevice->close ();
    return 0;
}


int Imu::savecalibration(){

  ofstream calfile;
  calfile.open ("/tmp/imucalibration.cal");
  calfile<<"Accelerometer"<<endl;
  for (int i=0;i<3;i++)
  {
    calfile << accmap[i]<< " ";
    calfile<< accaxis[i].min<<" ";
    calfile<< accaxis[i].max<<" ";
    calfile<< accaxis[i].zero<<" ";
    calfile<< accaxis[i].scale<<" ";
    calfile << endl;
    
  }
  calfile<<"Gyroscope"<<endl;
  
  for (int i=0;i<3;i++)
  {
    calfile << gyromap[i]<< " ";
    calfile<< gyroaxis[i].min<<" ";
    calfile<< gyroaxis[i].max<<" ";
    calfile<< gyroaxis[i].zero<<" ";
    calfile<< gyroaxis[i].scale<<" ";
    calfile << endl;
    
  } 
  calfile.close();
}

int Imu::loadcalibration(){
  cout <<"Loading calibration"<<endl;
  ifstream calfile;
  string input;
  calfile.open ("/tmp/imucalibration.cal");
  
  calfile>>input;cout << input<<endl;
  for (int i=0;i<3;i++)
  {
    calfile >> accmap[i];
    calfile >> accaxis[i].min;
    calfile >> accaxis[i].max;
    calfile >> accaxis[i].zero;
    calfile >> accaxis[i].scale;
    
  }
  calfile>>input;cout << input<<endl;
  for (int i=0;i<3;i++)
  {
    calfile >> gyromap[i];
    calfile >> gyroaxis[i].min;
    calfile >> gyroaxis[i].max;
    calfile >> gyroaxis[i].zero;
    calfile >> gyroaxis[i].scale;
    
    
  } 
  calfile.close();
}


int
Imu::imu_get_values ()
{
      if (!calibrated) gyro_calibrate_zero();

    int usecs=stopwatch.get_usecs();
    stopwatch.reset();
    usbdevice->get(0);
    /*int g[3];
    int a[3];*/
    gyroaxis[0].value = (int)usbdevice->gyro[0];
    gyroaxis[1].value = (int)usbdevice->gyro[1];
    gyroaxis[2].value = (int)usbdevice->gyro[2];
    accaxis[0].value  = (int)abs(usbdevice->acc[0]);
    accaxis[1].value  = (int)abs(usbdevice->acc[1]);
    accaxis[2].value  = (int)abs(usbdevice->acc[2]);
    fy = !usbdevice->slow[0];
    fp = !usbdevice->slow[1];
    fr = !usbdevice->slow[2];

    if (fy)
        gyroaxis[0].value *= 4.5f;
    if (fp)
        gyroaxis[1].value *= 4.5f;
    if (fr)
        gyroaxis[2].value *= 4.5f;


// acc x accy accz is vector!
//double ref_x=0,ref_y=0;ref_z=1;


//    cout << "\ny p r " << yaw << " " << pitch << " " << roll;
    if (fp&&debug)
        cout << "Fast PIT!!!";
    if (fr&&debug )
        cout << "Fast  ROLL!!!";
    if ( fy&&debug)
        cout << "Fast YAQ !!!";
  //  cout << endl;





    

    // Start integrating stuff
    Axis*axis;
    axis=&gyroaxis[gyromap[YAW]];
    yaw   = axis->scale*(axis->value - axis->zero) * 1.0f / 20;
    axis=&gyroaxis[gyromap[PITCH]];
    pitch = axis->scale*(axis->value - axis->zero) * 1.0f / 20;
    axis=&gyroaxis[gyromap[ROLL]];
    roll  = axis->scale*(axis->value - axis->zero) * 1.0f / 20;

    axis=&accaxis[accmap[ACCX]];
    acc_x=axis->scale * filter.convert2G (axis->min, axis->max, axis->zero, axis->value);
    axis=&accaxis[accmap[ACCY]];
    acc_y=axis->scale * filter.convert2G (axis->min, axis->max, axis->zero, axis->value);
    axis=&accaxis[accmap[ACCZ]];
    acc_z=axis->scale * filter.convert2G (axis->min, axis->max, axis->zero, axis->value);
    
    // DEGREES
    
    
    double a1,a2,a3;

   /* a1= filter.convert2G (267, 690, 495, acc_x);
    a2= filter.convert2G (299, 720, 511, acc_y);
    a3= -filter.convert2G (340, 780, 570, acc_z);
    */
    filter.g_x=acc_x;
    filter.g_y=-acc_z;
    filter.g_z=-acc_y;



if (debug)    printf("\nAccs: x %f ,y  %f, z %f",filter.g_x,filter.g_y,filter.g_z);




//double scale_gyro = 1;	//90*1.0f/195;;


// Orientation quaternion update:
    Quaternion qgyro;

//    roll=0;pitch=0;yaw=0;c
    dt_s=usecs*1.0f/1000000;
    //cout <<"dt "<<dt_s<<endl;

    qgyro.set (0, -pitch * dt_s * Deg2Rad, -yaw * dt_s * Deg2Rad,
               -roll * dt_s * Deg2Rad);

   // double gp, gy, gr;
    Quaternion delta;
    delta = (filter.gyro_int * qgyro) * 0.5f;
    filter.gyro_int = filter.gyro_int + delta;
//    filter.gyro_int.toEuler (&gp, &gy, &gr);



    filter.orient = filter.gyro_int;
    filter.orient = filter.orient.normalize ();





    Quaternion temp, rota;
    temp.set (0, filter.g_x, filter.g_y, filter.g_z);


    Quaternion lala;

    //rota = filter.orient.get_conjugate() * temp * filter.orient;
    rota = filter.orient * temp * filter.orient.get_conjugate();
    //rota = lala * temp * lala.get_conjugate();
    filter.think_x = rota.x;
    filter.think_y = rota.y;
    filter.think_z = rota.z;


if(debug)    cout << " rota x" << rota.x << " y " << rota.y << " z " << rota.z << endl;
    V3d int_3d (rota.x, rota.y, rota.z);
    V3d acc_3d (0, -1, 0);
//      cout << " rot2 x" << rota.x << " y " << rota.y << " z " << rota.
//	z << endl;
    int_3d = int_3d.normalize ();
    acc_3d = acc_3d.normalize ();
    axisrotate =acc_3d.crossprod (int_3d);
    axisrotate=axisrotate.normalize();
    axisrotate.x=-axisrotate.x;    
    axisrotate.y=-axisrotate.y;    
    axisrotate.z=-axisrotate.z;    
// cout <<" ax x"<< axis.x<<" y "<<axis.y<<" z "<<axis.z<<endl;
    double ang = acos ((int_3d.dotprod (acc_3d))/(acc_3d.length()*int_3d.length()));
if(debug)    cout << "The angle is " << ang / M_PI * 180 ;

    Quaternion rollback;
    rollback.fromAxisAngle (ang, axisrotate);

    Quaternion rotated;
    rotated = rollback*filter.orient*rollback;
    rotated.normalize();

    filter.gyro_int.normalize();
    
     double dotprod=filter.gyro_int.dot_product_with(rotated);
     if(debug)    cout << "The dotproduct is" << dotprod ;

     if (dotprod<0)rotated=rotated*-1;


    filter.gyro_int.nlerp (rotated, 0.01001f);

    filter.orient = filter.gyro_int;

    /*
        filter.g_x=rota.x;
       filter.g_y=rota.y;
       filter.g_z=rota.z;
    */

    filter.orient.toMatrix4x4 (filter.mat4x4);
//printf("QUAT %8.03f %8.03f %8.03f %8.03f",filter.orient.r,filter.orient.x,filter.orient.y,filter.orient.z);


    //filter.rkyaw = Rad2Deg*filter.yawRK.computeRK4(yaw*Deg2Rad*dt_s);
//cout<<endl<< "dt_s=<<"<<dt_s<<endl;
//   filter.kpitch=filter.kpitch*0.99+filter.accelAngleY*0.01;
    //  filter.kroll=filter.kroll*0.99+filter.accelAngleX*0.01;




    return 2;
}



//cout <<"nach scanff"<<endl;

//Motor v umrechnen
//cout<<"nach ser_receive"<<endl;








//----------------------------------------------------------------------------
#ifdef USE_USBIMU_MAIN

int
main (int argc,char **argv)
{
    string input;


    Imu imu;

    int i = 0;

    if ((argc>1)) {
        if ( !strcmp(argv[1],"calibrate")) {
            cout << "Ready for calibration? (y/n) :"<<endl;
            cin>>input;
            if (input=="y") {
                cout <<"\nCalibrating Gyro Zero first. Be still! any key and Enter to start"<<endl;
                cin>>input;
                imu.gyro_calibrate_zero();
		cout <<"\nLAY IMU Flat (calibrating accelerometer 0 )"<<endl; cin>>input;
                imu.accelerometer_calibrate(Imu::FLAT);
		cout <<"Turn IMU Right like a plane any key and Enter to start"<<endl;cin>>input;
                imu.accelerometer_calibrate(Imu::RIGHT);
		cout <<"\nTurn IMU Front like a plane diving and Enter to start"<<endl;cin>>input;
                imu.accelerometer_calibrate(Imu::FRONT);
		cout <<"\nLay IMU FLAT and ROTATE AROUND Earth AXIS Clockwise"<<endl;cin>>input;
                imu.accelerometer_calibrate(Imu::TURN);
		
		cout << "IMU CALIBRATED !!!!"<<endl;



            }
            else
                exit (0);

            cout<<"calibration done.. saving ...misses"<<endl;
	    
	    imu.savecalibration();
	    
            exit(0);
        }
	if ( !strcmp(argv[1],"load")) {
            
	    imu.loadcalibration();
	    
            exit(0);
        }
    }

    imu.loadcalibration();
	
    int count = 0;
    while (i < 1000000)
    {
        imu.imu_get_values ();
        count++;
            printf
            ("\n Raw: y%6.02f p%6.02f r%6.02f ax%6.02f ay%6.02f ta%6.02f kr%6.02f kp%6.02f rky%6.02f",
             imu.yaw, imu.pitch, imu.roll, imu.filter.accelAngleX,
             imu.filter.accelAngleY, imu.filter.theta * Rad2Deg,
             imu.filter.kroll, imu.filter.kpitch, imu.filter.rkyaw);

            i++;
    }


    return 0;
}


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
#endif

long
Imu::get_current_time ()
{
    timeval tval;
    if (gettimeofday (&tval, NULL))
        std::cerr << "\n something wrong with time mesurement";
    return tval.tv_sec * 1000 + tval.tv_usec / 1000;

}

long
Imu::get_current_usec ()
{
    timeval tval;
    if (gettimeofday (&tval, NULL))
        std::cerr << "\n something wrong with time mesurement";
    return tval.tv_usec;
}
