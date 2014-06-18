#include "CircleFilter.h"
#include "../Formation/Painter.h"

#include <cmath>
#include <stdlib.h>
#include "ColorClasses.h"


using namespace std;


namespace Tribots
{
  CircleFilter::CircleFilter()
  {
	  LineFilter::LineFilter();
  
  }


  void
  CircleFilter::scanLine(const Tribots::Image & img ,Vec start, Vec end,int kernel)
  { // start ist Mittelpunkt, end ist Punkt auf dem Kreis.

    Vec access;
    Vec one=(end-start).normalize();// radiales one
    Vec radius=(end-start);
    double length;
    length=radius.length()*M_PI*2;//umfang entfernung
    int steps=(int)length;
    access=end;
    double onerot=2.0f*M_PI/steps;
    double rot=0;
    //get Y pixels from image
    YUVTuple yuv;
    if (steps>999) steps=999;
           
    if (integrationwidth>50) integrationwidth=50; // soviel wollen wir auch nicht integrieren.
    if (integrationwidth==0){
    	for(int i=0;i<steps;i++)
    	{	rot+=onerot;
      		access=start+radius.rotate(Angle(rot));
      		img.getPixelYUV((int)access.x,(int)access.y,&yuv);
      		buffer[i]=yuv.y;
    	}
    }
    else{ // ok hier wird integriert. und normalisiert (wir ham ja zeit.)
 /*   Vec s1,s2,e1,e2;
    // 4 Punke berechnen.
    s1=start+one.rotate_quarter()*integrationwidth;
    e1=end+one.rotate_quarter()*integrationwidth;
    s2=start+one.rotate_quarter()*-integrationwidth;
    e2=end+one.rotate_quarter()*-integrationwidth;
    Vec onelateral=one.rotate_three_quarters();
    double laterallength=(s1-s2).length();
    int lateralsteps=(int)laterallength;
    // die vier punkte müssen im Bild sein. Ich nehme mal an dass getpixel keinen speichermüll zurückliefert.
    access=start;
    Vec lateralaccess;
    for (int i=0;i<steps;i++){//entlang der line
	    access=access+one;
	    lateralaccess=access;
	    buffer[i]=0;
	    for (int j=0;j<lateralsteps;j++){
		    lateralaccess=lateralaccess+onelateral;
		    img.getPixelYUV((int)lateralaccess.x,(int)lateralaccess.y,&yuv);
		    buffer[i]=buffer[i]+yuv.y;
	    }
	    //normalisieren
	    buffer[i]=buffer[i]/lateralsteps;
    }
   */


    
    }

    num_buffer=steps; // speichern der anzahl der Grauwerte;
    //verschiedene kernels:

    if (kernel==MP)
      for(int i=1;i<steps-1;i++)
      {
        fbuffer[i]=-buffer[i-1]+buffer[i];
        fbuffer[i]*=0.5;

      }
    if (kernel==MOP)
      for(int i=1;i<steps-1;i++)
      {
        fbuffer[i]=-buffer[i-1]+buffer[i+1];
        fbuffer[i]*=0.5;
      }
    if (kernel==MMPP)
      for(int i=2;i<steps-2;i++)
      {
        fbuffer[i]=-buffer[i-2]-buffer[i-1]+buffer[i]+buffer[i+1];
        fbuffer[i]*=0.25f;

      }

    if (kernel==MMMPPP)
      for(int i=3;i<steps-3;i++)
      {
        fbuffer[i]=-buffer[i-3]-buffer[i-2]-buffer[i-1]+buffer[i]+buffer[i+1]+buffer[i+2];
        fbuffer[i]*=1.0f/6;
      }

    if (kernel==MMMMPPPP)
      for(int i=4;i<steps-4;i++)
      {
        fbuffer[i]=-buffer[i-4]-buffer[i-3]-buffer[i-2]-buffer[i-1]+buffer[i]+buffer[i+1]+buffer[i+2]+buffer[i+3];
        fbuffer[i]*=0.125;
      }

    num_fbuffer=steps; // das gefilterte array ist genauso aligned wie das ungefilterte.
    // durch das array wandern und die wendepunkte bestimmen;

    int state=0;   /// 0 +1 oder -1 möglich
    int max;max=0;
    int maxpos;maxpos=0;
    int c=0;
    for(int i=0;i<steps;i++)
    {
      if (state==0&&abs(fbuffer[i])<3) continue; //state bleibt gleich 0 !!

      if (state<0&&fbuffer[i]>-3)
      {
        state=0;
        if(abs(max)>3){results[c]=maxpos;c++;max=0;}
      }
      if (state>0&&fbuffer[i]<3)
      {
        state=0;
        if(abs(max)>3){results[c]=maxpos;c++;max=0;}
      }



      if (state>0&&fbuffer[i]>max){max=fbuffer[i];maxpos=i;}
      if (state<0&&fbuffer[i]<max){max=fbuffer[i];maxpos=i;}
      // derzeitigen state bestimmen
      if (fbuffer[i]<0)state=-1;else state=+1;
      if (abs(fbuffer[i])<3)state=0;
    }
    num_results=c;
    int lnum=0;
    int lastval=0;
    if (c>0)lastval=fbuffer[results[0]];
    int val;
    for (int i=0;i<c;i++)
    {
      val=fbuffer[results[i]];
      if ( (lastval >0)&&(val<0)&&(lastval+val)<fabs(val/3))
      {
        lines[lnum]=results[i-1]+0.5f*(results[i]-results[i-1]);
        lnum++;
      }
      lastval=val;
    }
    num_lines=lnum;


  }

  void   CircleFilter::visualize(Tribots::Image & img ,Vec start, Vec end)
  {
    Tribots::Painter p (img);
    Tribots::RGBTuple col;
    col.g=150;
    col.b=150;
    col.r=100;
    p.setColor (col);
    p.drawCircle(start,(start-end).length());
    Vec one=(end-start).normalize();
    //c  Maxima gefunden.
    col.g=0;col.b=255;col.r=0;
    p.setColor (col);
    Vec po,po2;
    Vec radius;
    radius=end-start;
    double length=radius.length()*M_PI*2;//umfang entfernung
    int steps=(int)length;
    double onerot=2.0f*M_PI/steps;
    double rot=0;
    
 Vec access=radius;
    for(int i=0;i<num_results;i++)
    {
      po=start+radius.rotate(Angle(onerot*results[i]));;
      po2=po+(radius.normalize())*4;
      if (fbuffer[results[i]]>0)po2=po+(radius.normalize())*-4;

      p.drawLine(po,po2);
    }
    col.g=0;
    col.b=0;
    col.r=255;
    p.setColor (col);

    for (int i=0;i<num_lines;i++)
    {
      po=start+radius.rotate(Angle(onerot*lines[i]));
      po2=po+ radius.normalize()*2;
      po=po+ radius.normalize()*-2;
      p.drawLine(po,po2);
    }

  }



};



