#include <iostream>

#include "../../components/ImageWidget.h"
#include "../../../ImageProcessing/Formation/ImageProducer.h"
#include "../../../ImageProcessing/Formation/MultiImageProducer.h"
#include "../../../ImageProcessing/Formation/Image.h"
#include "../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../ImageProcessing/Formation/YUVImage.h"
#include "../../../Fundamental/ConfigReader.h"
#include "../../../ImageProcessing/PixelAnalysis/Image2WorldMapping.h"
#include "../../../ImageProcessing/PixelAnalysis/DirectionalCameraMapping.h"
#include "../../../Fundamental/Vec.h"
#include "../../../Fundamental/geometry3D.h"
#include "../../../WorldModel/WorldModel.h"
#include "../../../ImageProcessing/ImageProcessing.h"
#include "../../../ImageProcessing/Formation/ImageIO.h"
#include "../../../ImageProcessing/Formation/PPMIO.h"

using namespace Tribots;
using namespace TribotsTools;
using namespace std;

#define WARP_WIDTH 10000
#define WARP_HEIGHT 10000
#define MAX_CAMS 8

#define WIN_SIZE_X 500
#define WIN_SIZE_Y 500

class CClickImageWidget : public ImageWidget{
 public:
  ImageWorldMapping* lut;

  CClickImageWidget::CClickImageWidget(Image &mapImage, 
				       ImageWorldMapping* lut) 
    : ImageWidget(mapImage), lut(lut)
  {}

  void CClickImageWidget::mousePressEvent ( QMouseEvent * e ){
     cout << e->x() << " " << e->y() << " --> ";
     Vec vic = lut->map(e->x(), e->y());
     cout << vic.x << " "<<vic.y << endl;
  }

};

int main(int argc, char* argv[])
{
  ImageWorldMapping* lut;
  WorldImageMapping* lutw2i;
  if (argc<=1)
  {
    cerr << "Aufruf: " << argv[0] << " PPM-Datei Directional.cfg" << endl;
    return -1;
  }

  int arg_cnt=1;
  
  while(argc>++arg_cnt && argv[arg_cnt][0]=='-'){
/*     switch(argv[arg_cnt][1]){
       case 'm' :  break;
       default: cerr << " Unknown parameter : "<< argv[arg_cnt] << endl;
     }
*/  }
  

  try{

    //---Initializations
    
    //multiple images
    Image* newImage;
    Image* mapImage;

    CClickImageWidget* widget;
    CClickImageWidget* mapwidget;
    //Initialize QApplication
    QApplication app(argc, argv);	//only one QApplication is allowed.

    // get image from Imageprocessing or ImageProducer
	ImageIO *imageIO=new PPMIO;
	ImageBuffer* buffer;
	try {
		cout << "going to read " << argv[1] << "\n";
		buffer=imageIO->read(0,argv[1]);
	}
	catch (TribotsException& e)
	{
		cerr << "Exception: " << e.what() << "\n";
	}

    newImage = new RGBImage(*buffer);	//load first images from different sources


  if(argc-arg_cnt > 0)
  {
    //ImageWorldDirectionalMapping 
    lut = new ImageWorldDirectionalMapping(argv[arg_cnt],newImage->getWidth(),newImage->getHeight(),0,0);
    lutw2i = new WorldImageDirectionalMapping(argv[arg_cnt],newImage->getWidth(),newImage->getHeight(),0,0);
//  lut = iw_directed.convert();
  }

//	widget = new CClickImageWidget(*newImage, lut);
//	widget->show();
	
	  mapImage = new RGBImage(WIN_SIZE_X, WIN_SIZE_Y);
	  mapwidget = new CClickImageWidget(*mapImage, lut);
	  mapwidget->show();
	
    app.setMainWidget(mapwidget);

    QObject::connect( &app, SIGNAL(lastWindowClosed()),
		      &app, SLOT(quit()) );

    //Main loop that shows the images.
    //Active until window is closed.

    Vec target, trans(16,16);
    RGBTuple line;
    line.r=line.g=0; line.b=255;

    RGBTuple pixel;

    // map image to world coordinates, if lut was specified for this cam
    if(lut!=NULL)
		{
      int width=newImage->getWidth(), height=newImage->getHeight();
      Vec origin(mapImage->getWidth()/2,mapImage->getHeight());
	   
      for(int x=0; x<mapImage->getWidth(); ++x)
        for(int y=0; y<mapImage->getHeight(); ++y)
        {
          Vec World( ((x-mapImage->getWidth()/2) * trans.x),(mapImage->getHeight()-y) * trans.y);
        	target = lutw2i->map(World);
  				if( target.x>=0 && target.x<width
				    &&target.y>=0 && target.y<height)
	   			{
  					newImage->getPixelRGB(static_cast<int>(target.x), static_cast<int>(target.y), &pixel);
  					mapImage->setPixelRGB(x,y, pixel);
  				}
			  }
        
        /*
        // y-achse (grundlinie)
        for(int y=0; y<mapImage->getHeight(); ++y)
        {
          // grundlinie
          mapImage->setPixelRGB((int)(origin.x-(50/trans.x)),y, line);
          mapImage->setPixelRGB((int)(origin.x+(50/trans.x)),y, line);
        
          // goalarea linie
          mapImage->setPixelRGB((int)(origin.x+(525/trans.x)),y, line);
          mapImage->setPixelRGB((int)(origin.x+(475/trans.x)),y, line);              
          
          // penaltyarea line
          mapImage->setPixelRGB((int)(origin.x+(1525/trans.x)),y, line);
          mapImage->setPixelRGB((int)(origin.x+(1475/trans.x)),y, line);
        }
        // x-achse
        for (int x=0; x < mapImage->getWidth();++x)
        {
         // penalty area line (near)
          mapImage->setPixelRGB(x,(int)(origin.y-(1285/trans.y)), line);            
          mapImage->setPixelRGB(x,(int)(origin.y-(1235/trans.y)), line);            
          // goal area line (near)
          mapImage->setPixelRGB(x,(int)(origin.y-(1785/trans.y)), line);            
          mapImage->setPixelRGB(x,(int)(origin.y-(1735/trans.y)), line);            
          // goal area line (far)
          mapImage->setPixelRGB(x,(int)(origin.y-(4785/trans.y)), line);            
          mapImage->setPixelRGB(x,(int)(origin.y-(4735/trans.y)), line);            
          // penalty area (far)
          mapImage->setPixelRGB(x,(int)(origin.y-(5285/trans.y)), line);            
          mapImage->setPixelRGB(x,(int)(origin.y-(5235/trans.y)), line);            
          // bounding line (far)
          mapImage->setPixelRGB(x,(int)(origin.y-(6300/trans.y)), line);            
          mapImage->setPixelRGB(x,(int)(origin.y-(6200/trans.y)), line);            
        }*/
        // y-achse
        for(int y=0; y<mapImage->getHeight(); ++y)
        {
          // torpfosten rechts
          mapImage->setPixelRGB((int)(origin.x+(950/trans.x)), y, line);
          mapImage->setPixelRGB((int)(origin.x+(1050/trans.x)), y, line);
          // torpfosten links
          mapImage->setPixelRGB((int)(origin.x-(950/trans.x)), y, line);
          mapImage->setPixelRGB((int)(origin.x-(1050/trans.x)), y, line);          
          // goalarea rechts
          mapImage->setPixelRGB((int)(origin.x+(1700/trans.x)), y, line);
          mapImage->setPixelRGB((int)(origin.x+(1800/trans.x)), y, line);          
          // goalarea links
          mapImage->setPixelRGB((int)(origin.x-(1700/trans.x)), y, line);
          mapImage->setPixelRGB((int)(origin.x-(1800/trans.x)), y, line);          
          // penaltyarea rechts
          mapImage->setPixelRGB((int)(origin.x+(3200/trans.x)), y, line);
          mapImage->setPixelRGB((int)(origin.x+(3300/trans.x)), y, line);                    
          // penaltyarea links
          mapImage->setPixelRGB((int)(origin.x-(3200/trans.x)), y, line);
          mapImage->setPixelRGB((int)(origin.x-(3300/trans.x)), y, line);                    
          // "mittlere linie"
          mapImage->setPixelRGB((int)(origin.x), y, line);                    
        }
        // x-achse
        for (int x=0; x < mapImage->getWidth();++x)
        {
          // grundlinie
          mapImage->setPixelRGB(x,(int)(origin.y-(7050/trans.y)), line);
          mapImage->setPixelRGB(x,(int)(origin.y-(6950/trans.y)), line);
          // goalarea linie
          mapImage->setPixelRGB(x,(int)(origin.y-(6200/trans.y)), line);
          mapImage->setPixelRGB(x,(int)(origin.y-(6300/trans.y)), line);          
          // penaltyarea linie
          mapImage->setPixelRGB(x,(int)(origin.y-(4700/trans.y)), line);
          mapImage->setPixelRGB(x,(int)(origin.y-(4800/trans.y)), line);          
          // "penalty point" linie
          mapImage->setPixelRGB(x,(int)(origin.y-(4000/trans.y)), line);          
        }
    	}
      
  mapwidget->setImage(*mapImage);

    while (app.mainWidget()->isVisible()) {
		//war vorher am Ende:
		app.processEvents();

    }  //end while loop
	
  	delete newImage;	

        delete mapImage;
    
  }catch(Tribots::InvalidConfigurationException& e){
    cerr << e.what() << "\n\r" << flush;
  }catch(Tribots::TribotsException& e){
    cerr << e.what() << "\n\r" << flush;
  }catch(std::exception& e){
    cerr << e.what() << "\n\r" << flush;
  }
  return 0;
}
