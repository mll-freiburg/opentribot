#include "LineFilter.h"
#include "../Formation/Painter.h"
#include <cmath>
#include "ColorClasses.h"
#include "../Types/ScanLineImageProcessing.h"
#include "stdlib.h"



namespace Tribots
{
  LineFilter::LineFilter()
  {
  integrationwidth=0; // Es wird nicht integriert normalerweise.
  scanresultlist=0;  

	  for(int i=0;i<1000;i++)
    {
      buffer[i]=0;
      fbuffer[i]=0;
      results[i]=0;
      lines[i]=0;

    }

  }


  void
  LineFilter::scanLine(const Tribots::Image & img ,Vec start, Vec end,int kernel)
  {
    Vec access;
    Vec intaccess;
    Vec one=(end-start).normalize();
    double length;
    length=(end-start).length();
    int steps=(int)length;
    access=start;
    
    //get Y pixels from image
    YUVTuple yuv;

   const ColorClassifier * colorclassifier = img.getClassifier();
   char colorclass = COLOR_IGNORE;


    bool publish=!(scanresultlist==0);
   
    // ACHTUNG: Hindernishack fuer RoboCup Suzhou eingefuegt. Testen und Parameter zukuenftig aus Konfigurationsdateien beachten!

    bool findblack=true;                    // Hindernisse suchen
    if (steps>999) steps=999;
    if (integrationwidth>50) integrationwidth=50; // soviel wollen wir auch nicht integrieren.
    if (integrationwidth==0) {              // dieser Fall wird verwendet. es muessen auch baelle und hindernisse gefunden werden. TODO: Im Moment noch statisch reingehackt, ohne auf colorclass beschreibung zu achten.
      bool seenFirstObstacle = false;       // es wird nur das erste Hindernis weitergereicht
      int oldColorClass = COLOR_IGNORE;
      Vec oldPos = access;
      bool first=true;                      // first pixel on scanline
      bool onlyFirst = true;
      bool virtualTransition = false;
      bool scanBlackPixelNeighbourhood = false; // im konstruktor uebergeben; kommt aus config-file
      for(int i=0;i<steps;i++) {
        access=access+one;  // bloed, da so gerade das erste pixel auf der scanlinie ignoriert wird...
        intaccess.x=(int)access.x;
        intaccess.y=(int)access.y;
		
        img.getPixelYUV((int)access.x,(int)access.y,&yuv);
        colorclass=colorclassifier->lookup(yuv);
        if (colorclass==COLOR_BALL) {
          if (publish) scanresultlist->results[COLOR_BALL]->points.push_back(intaccess);
        }
        if (oldColorClass != colorclass && findblack && (!seenFirstObstacle || !onlyFirst) && publish && (colorclass==COLOR_OBSTACLE || oldColorClass==COLOR_OBSTACLE)) {
          if (oldColorClass == COLOR_OBSTACLE) { // Enduebergang
            Vec twoPos = access + one;
            scanresultlist->results[COLOR_OBSTACLE]->transitions.push_back(
              Transition(Transition::END,
                         oldColorClass, colorclass, 
                         img.getPixelClass(static_cast<int>(twoPos.x),
                                           static_cast<int>(twoPos.y)),
                         oldPos, access, virtualTransition)); // first is a virtual trans
            seenFirstObstacle = true;
          }
          else { // colorclass == COLOR_OBSTACLE
            virtualTransition = first; // is this a transition at the first pixel of the scanline? if so, mark as virtual, because the black region may have started earlier
            Vec twoPos = oldPos - one;            
            bool accept_transition = true;
            if (scanBlackPixelNeighbourhood 
                && static_cast<int>(access.x)>1
                && static_cast<int>(access.x)<img.getWidth()-1
                && static_cast<int>(access.y)>1
                && static_cast<int>(access.y)<img.getHeight()-1
                ) {
              int col_bin=0;
              accept_transition = false;
              for (int dx = -1 ; dx <= 1 ; dx++)
                for (int dy = -1 ; dy <= 1 ; dy++) {
                  if (img.getPixelClass(static_cast<int>(access.x+dx),
                                          static_cast<int>(access.y+dy))==COLOR_OBSTACLE) col_bin++;
                }
              if (col_bin>=3) accept_transition = true;
            }
            if (accept_transition) {
              scanresultlist->results[colorclass]->transitions.push_back(
                Transition(Transition::START,
                           oldColorClass, colorclass,
                           img.getPixelClass(static_cast<int>(twoPos.x),
                                               static_cast<int>(twoPos.y)),
                           oldPos, access, virtualTransition));
            }
          }
        }
        buffer[i]=yuv.y;
        first = false;
        oldPos = access;
        oldColorClass = colorclass;
    	}
      if (colorclass == COLOR_OBSTACLE && findblack && (!seenFirstObstacle || !onlyFirst) && publish) { // last pixel was in black object. have to store end transition
        scanresultlist->results[colorclass]->transitions.push_back(
          Transition(Transition::END,
                     colorclass, COLOR_IGNORE, COLOR_IGNORE,
                     oldPos, access+one, true));  // last is a virtual transition
      }
    }
    else{ // ok hier wird integriert. und normalisiert (wir ham ja zeit.)
    Vec s1,s2,e1,e2;
	
    // 4 Punke berechnen.
    s1=start+one.rotate_quarter()*integrationwidth;
    e1=end+one.rotate_quarter()*integrationwidth;
    s2=start+one.rotate_quarter()*-integrationwidth;
    e2=end+one.rotate_quarter()*-integrationwidth;
    Vec onelateral=one.rotate_three_quarters();
    double laterallength=(s1-s2).length();
    int lateralsteps=(int)laterallength;
    // die vier punkte m¸ssen im Bild sein. Ich nehme mal an dass getpixel keinen speicherm¸ll zur¸ckliefert.
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
   


    
    }




    num_buffer=steps; // speichern der anzahl der Grauwerte;
    //verschiedene kernels:
    //internal factor for fbuffer => der grenzwert muss um fac hoeher sein
    double fac;

    if (kernel==MP){
	    fac=2;
      for(int i=1;i<steps-1;i++)
      {
        fbuffer[i]=-buffer[i-1]+buffer[i];
      }
    }
    if (kernel==MOP){
	    fac=2;
      for(int i=1;i<steps-1;i++)
      {
        fbuffer[i]=-buffer[i-1]+buffer[i+1];
      }
    }
    if (kernel==MMPP){
	    fac=4;
      for(int i=2;i<steps-2;i++)
      {
        fbuffer[i]=-buffer[i-2]-buffer[i-1]+buffer[i]+buffer[i+1];
      }
    }
    if (kernel==MMMPPP){
	    fac=6;
      for(int i=3;i<steps-3;i++)
      {
        fbuffer[i]=-buffer[i-3]-buffer[i-2]-buffer[i-1]+buffer[i]+buffer[i+1]+buffer[i+2];
      }
    }
    if (kernel==MMMMPPPP){
	    fac=8;
      for(int i=4;i<steps-4;i++)
      {
        fbuffer[i]=-buffer[i-4]-buffer[i-3]-buffer[i-2]-buffer[i-1]+buffer[i]+buffer[i+1]+buffer[i+2]+buffer[i+3];
      }
    }
    num_fbuffer=steps; // das gefilterte array ist genauso aligned wie das ungefilterte.
    // durch das array wandern und die wendepunkte bestimmen;

    int state=0;   /// 0 +1 oder -1 mˆglich
    int max;max=0;
    int maxpos;maxpos=0;
    int c=0;
    int thresh=4;

    for(int i=0;i<steps;i++)
    {
      if (state==0&&abs(fbuffer[i])<fac*thresh) continue; //state bleibt gleich 0 !!

      if (state<0&&fbuffer[i]>-thresh*fac)
      {
        state=0;
        if(abs(max)>thresh*fac){results[c]=maxpos;c++;max=0;}
      }
      if (state>0&&fbuffer[i]<thresh*fac)
      {
        state=0;
        if(abs(max)>thresh*fac){results[c]=maxpos;c++;max=0;}
      }



      if (state>0&&fbuffer[i]>max){max=fbuffer[i];maxpos=i;}
      if (state<0&&fbuffer[i]<max){max=fbuffer[i];maxpos=i;}
      // derzeitigen state bestimmen
      if (fbuffer[i]<0)state=-1;else state=+1;
      if (abs(fbuffer[i])<3*fac)state=0;
    }

    Vec pos;
    for(int i=0;i<c;i++){ // resultpos f¸r sp‰ter zwischenspeichern
     resultpos[i]= start+one*results[i];
     
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

	if (publish )scanresultlist->results[COLOR_LINE]->transitions.push_back(
	      		Transition(Transition::START,
			 COLOR_FIELD, COLOR_LINE,
			 COLOR_FIELD,
			 resultpos[i-1],resultpos[i-1])); // first is a virtual trans
	  	
	if (publish )	scanresultlist->results[COLOR_LINE]->transitions.push_back(
	      	Transition(Transition::END,
			 COLOR_LINE, COLOR_FIELD,
			 COLOR_FIELD,
			 resultpos[i],resultpos[i])); // first is a virtual trans





      }
      lastval=val;
    }
    num_lines=lnum;

  }

  void   LineFilter::visualize(Tribots::Image & img ,Vec start, Vec end)
  {
    Tribots::Painter p (img);
    Tribots::RGBTuple col;
    col.g=150;
    col.b=150;
    col.r=100;
    p.setColor (col);
    p.drawLine(start,end);
    Vec one=(end-start).normalize();
    //c  Maxima gefunden.
    col.g=0;col.b=255;col.r=0;
    p.setColor (col);
    Vec po,po2;

    for(int i=0;i<num_results;i++)
    {

      po=start+one*results[i];
      po2=po+(one.rotate_quarter())*3;
      if (fbuffer[results[i]]>0)po2=po+(one.rotate_three_quarters())*3;

      p.drawLine(po,po2);
    }
    col.g=0;
    col.b=0;
    col.r=255;
    p.setColor (col);

    for (int i=0;i<num_lines;i++)
    {
      po=start+one*lines[i];
      po2=po+ one.rotate_quarter()*2;
      po=po+ one.rotate_three_quarters()*2;
      p.drawLine(po,po2);
    }

  }


    void LineFilter::setIntegrationWidth(int width){

    integrationwidth=width;
    }
    ;
    
    
    void LineFilter::setScanResultList(ScanResultList* list){
	scanresultlist=list;

    }
    ;

};



