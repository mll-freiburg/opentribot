#include "vtkCubeSource.h"
#include "vtkLineSource.h"
  #include "vtkPolyDataMapper.h"
  #include "vtkRenderWindow.h"
  #include "vtkCamera.h"
  #include "vtkActor.h"
  #include "vtkRenderer.h"
  #include "vtkAssembly.h"
  #include "udpsocket.h"
#include "vtkMatrix4x4.h"
#include "vtkProperty.h"


int QuatToMatrix(double w,double x,double y,double z, double m[4][4])
{
float wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

// calculate coefficients used for building the matrix
 x2 = x + x; y2 = y+ y;
 z2 = z + z;
 xx = x * x2; xy = x * y2; xz = x * z2;
 yy = y * y2; yz = y * z2; zz = z * z2;
 wx = w * x2; wy = w * y2; wz = w * z2;
//
// // fill in matrix positions with them
 m[0][0] = 1.0 - (yy + zz); m[1][0] = xy - wz;
 m[2][0] = xz + wy; m[3][0] = 0.0;
 m[0][1] = xy + wz; m[1][1] = 1.0 - (xx + zz);
 m[2][1] = yz - wx; m[3][1] = 0.0;
 m[0][2] = xz - wy; m[1][2] = yz + wx;
 m[2][2] = 1.0 - (xx + yy); m[3][2] = 0.0;
 m[0][3] = 0; m[1][3] = 0;
 m[2][3] = 0; m[3][3] = 1;
 }







  int main() {
    vtkCubeSource *cube = vtkCubeSource::New();
    cube->SetXLength( 2.0 );
    cube->SetYLength( 3.0 );
    cube->SetZLength( 1.0 );


    vtkLineSource *line =vtkLineSource::New();
    line->SetPoint1(0,0,0);
    line->SetPoint2(0,0,-2);
    vtkLineSource *up =vtkLineSource::New();
    up->SetPoint1(0,0,0);
    up->SetPoint2(0,3,0);
    vtkLineSource *axis =vtkLineSource::New();
    axis->SetPoint1(0,0,0);
    axis->SetPoint2(0,3,0);

    vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
    mapper->SetInputConnection(line->GetOutputPort());
    vtkActor *lineActor=vtkActor::New();
    lineActor->SetMapper(mapper);


    vtkPolyDataMapper*upMapper=vtkPolyDataMapper::New();
    upMapper->SetInputConnection(up->GetOutputPort());
    vtkActor *uplineActor=vtkActor::New();
    uplineActor->SetMapper(upMapper);
    
    vtkPolyDataMapper*axisMapper=vtkPolyDataMapper::New();
    axisMapper->SetInputConnection(axis->GetOutputPort());
    vtkActor *axislineActor=vtkActor::New();
    axislineActor->SetMapper(axisMapper);

    vtkPolyDataMapper *coneMapper = vtkPolyDataMapper::New();
    coneMapper->SetInputConnection( cube->GetOutputPort() ); 


    vtkActor *coneActor = vtkActor::New();
    coneActor->SetMapper( coneMapper );
    vtkActor *coneActor2 = vtkActor::New();
    coneActor2->SetMapper( coneMapper );
//    coneActor->AddMapper(upMapper);    


   vtkAssembly *gyro=vtkAssembly::New();

   gyro->AddPart(coneActor);
   gyro->AddPart(lineActor);
//   gyro->AddPart(uplineActor);

    vtkRenderer *ren1= vtkRenderer::New();
    ren1->AddActor( coneActor );
    ren1->AddActor( coneActor2 );
    ren1->AddActor( uplineActor );
    ren1->AddActor( axislineActor );
    ren1->AddActor( gyro );
    ren1->SetBackground( 0.1, 0.2, 0.4 );

    vtkRenderWindow *renWin = vtkRenderWindow::New();
    renWin->AddRenderer( ren1 );
    renWin->SetSize( 300, 300 );

   /* int i;
    for (i = 0; i < 360; ++i) {
      renWin->Render();
      ren1->GetActiveCamera()->Azimuth( 1 );
    }
*/

  ren1->GetActiveCamera()->SetPosition(0,10,10);

 // coneActor->SetPosition(-2,0,0);
  coneActor2->SetPosition(+2,0,0);
  coneActor->GetProperty()->SetOpacity(0.4);
  coneActor2->GetProperty()->SetOpacity(0.4);
  coneActor->GetProperty()->SetColor(0,1,0);
  coneActor2->GetProperty()->SetColor(1,0,0);
UDPsocket sock;
  sock.init_socket_fd (58585);
  double * serverbuffer;
  serverbuffer = new double[500];
  memset(serverbuffer,0,500);
int len;
double ax,ay,az,p,r,x,y,z;
double q[4];


int i=0;
   while (true){
	i++;
	

sock.recv_msg((char*)serverbuffer, len,500,true);
cout <<" Igot message"<<endl;
       x=(serverbuffer[16]);
       y=(serverbuffer[17]);
       z=(serverbuffer[18]); 
       double xa=(serverbuffer[20]);
       double ya=(serverbuffer[21]);
       double za=(serverbuffer[22]); 
 


    
// vtkMath::QuaternionToMatrix3x3(q,rot_mat);

    //
    //coneActor->RotateWXYZ(q[0],q[1],q[2],q[3]);
/*    double rad2deg=180.0f/M_PI;    
    double sqw=q[0]*q[0];
    double sqx=q[1]*q[1];
    double sqy=q[2]*q[2];
    double sqz=q[3]*q[3];
     


    ax=rad2deg*atan2(  (2.f* (q[0]*q[3]+ q[1]*q [2])),sqx-sqy-sqz+sqw);    
    ay=rad2deg*asinf(  (2.f* (q[0]*q[2]- q[3]*q [1])));    
    az=rad2deg*atan2(  (2.f* (q[0]*q[1]+ q[2]*q [3])),-sqx-sqy+sqz+sqw);    

    ay=0;
*/



 vtkMatrix4x4 * mat;// = vtkMatrix4x4::New();
mat=	    gyro->GetMatrix();
mat->DeepCopy(serverbuffer);
// mat->Identity();

//coneActor->setUserMatrix
cout <<"e "<<i<<" "<<mat[0]<<endl;
cout <<"x y z "<<x <<" "<<y<<" "<<z<<endl;

    //coneActor->SetMatrix(mat);
 //   coneActor->SetOrientation(-ay,-az,-ax);
//

//	cout <<"rotating"<< ax<<"rotating"<< ay<<"rotating"<< az<<endl;
//    coneActor2->RotateWXYZ(q[0],q[1],q[2],q[3]);//-y,-z,-x);
    //
    //
    //yyconeActor->AddPosition(((serverbuffer[0]-500)*1.0f/(600)) , 0,0);
   // coneActor->TranslateZ((serverbuffer[0]-500)*1.0f/(60))

//    lineActor->RotateX(i);
 
cout <<" length"<<sqrt(x*x+y*y+z*z)<<endl;

up->SetPoint2(x*3,y*3,z*3);
axis->SetPoint2(1*xa,1*ya,1*za);
     renWin->Render();
		 


}
    cube->Delete();
    coneMapper->Delete();
    coneActor->Delete();
    coneActor2->Delete();
    ren1->Delete();
    renWin->Delete();

    return 0;
  }


