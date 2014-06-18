#include "CalibTool.h"

#include "../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../ImageProcessing/Formation/RGBImage.h"

#define U_DIM 128
#define V_DIM 128
#define SCALE 5

namespace Tribots{

  using namespace TribotsTools;

/**
 * ImageWidget that allows the user to perform certain actions in the image.
 * These are:
 *	- Zoom the area undnerneath the mouse pointer.
 *	- Add a pixel's coordinates to a list by click of the left mouse button.
 *	- Hold the current zoomed view to aim more precisely by holding the Ctrl-Key pressed.
 *		--> Yields sub-pixel accuracy.
 *	- Remove coordinates of the last pixel from that list by click of the right mouse button.
 *	- Save those coordinates to the file given in the params by pressing enter. Advances
 *	  to the next .ppm picture.
 *      - The Left- and Right-key can flip through the .ppm files stored in the given directory.
 *
 *
 *   Authors: Christopher Loerken, cloerken@uos.de
 *            Tobias Kringe, tkringe@uos.de
 *
 *   Date: 16.06.2005
 */
class MyClickWidget : public ImageWidget{


protected :
CalibTool* calib;

public :

Tribots::MyClickWidget::MyClickWidget(CalibTool* caliber, const Image& image,
				    QWidget* parent = 0,
				    WFlags f = WType_TopLevel)
  : ImageWidget(image, parent, f)
{
        calib = caliber;
        //With mousetracking enabled, this method is called everytime
	//when the mouse is being moved. Otherwise the mouse would have to
	//be pressed as well.
	QWidget::setMouseTracking(true);
}


void MyClickWidget::mouseMoveEvent (QMouseEvent *e){
   QPoint pos = e->pos();
   calib->zoom(pos.x(), pos.y());
   setImage(*(calib->getImage()));
}

void MyClickWidget::mousePressEvent (QMouseEvent *e){
   QPoint pos = e->pos();
   if (e->button() == LeftButton)
      calib->setPixel(pos.x(), pos.y());
   else if (e->button() == RightButton)
      calib->removeLastPixel();
   setImage(*(calib->getImage()));
}

void MyClickWidget::keyPressEvent ( QKeyEvent * e ){
   if (e->key() == Key_Return)
      calib->save();
   else if (e->key() == Key_Control) //Trigger moving within zoomed area
      calib->setHighPrecision(true);
   else if (e->key() == Key_Left){
	   calib->lastImage();
	   
   } else if (e->key() == Key_Right){
	   calib->nextImage();
   }
   else (e->ignore());
   setImage(*(calib->getImage()));
}

void MyClickWidget::keyReleaseEvent ( QKeyEvent * e ){
   if (e->key() == Key_Control) //Trigger moving within zoomed area off
      calib->setHighPrecision(false);
   else (e->ignore());
}

}; // end MyClickWidget


//----------------------------------------- Calib Tool ---------------------------------------------

CalibTool::CalibTool(int argc, char* argv[]) :  pointc(0), 
						anz(0), 
						image(NULL), 
						originalImage(NULL),
						precision(false),
						hdir(NULL),
						current_entry(0) {
    if (argc!=4 && argc != 5){
       throw TribotsException("Invalid initialization of CalibTool object.");
    }
    output_file = argv[2];
    
    //read ppm image
    if (argc == 4) { //only one image specified
      readImage(argv[3]);
    } else if (strcmp(argv[3], "-d") == 0) {	//Read directory of images.
      hdir = opendir(argv[4]);			//Open directory and get handle.    
      dirname = argv[4] + string("/");
      nextImage();			//Get first image.    
    }else { 
    	CalibTool::usage();    
	exit(0);
    }    
    readChessboard(argv[1]);
    init(argc, argv);
}

CalibTool::~CalibTool(){
    for (int i = 0 ; i < anz ; i++)
    	delete[] points[i];
    delete[] points;
    delete widget;
    closedir(hdir);
}

void CalibTool::readImage(const char* image_name){
   ifstream in(image_name);
   PPMIO io;
   //cout << "ImageRead: Stream opened" << endl << flush;
   if (image) delete image;                 // delete old image;
   image =  new RGBImage( *io.read(NULL, in) );
   in.close();
   //cout << "ImageRead: read ppm-image" << endl << flush;
   
   //store original image, used for zooming-effect
   if (originalImage) delete originalImage; // delete old image
   originalImage = image->clone();
   //cout << "ImageRead: clone successful." << endl << flush;
}


/** Help functions that works like a contains() function for a vector<long> */
bool has_entry(vector<long>* ppm_entries, long l){
	vector<long>::iterator it = ppm_entries->begin();
	while (it != ppm_entries->end()){
		if (*it == l) 
			return true;
		it++;
	}
	return false;
}
  

bool CalibTool::nextImage(){
    //TODO: Exceptionhandling
   struct dirent *entry;
   if (ppm_entries.empty() || current_entry == ppm_entries.size()-1)	//--> read new image 
   {
    do { //Loop for ppm image
	    long remember = telldir(hdir);
	    entry = readdir(hdir); //sets stream to next image already... Therfore remember old id of iamge.
        if (entry != NULL){
    	   if (strstr(entry->d_name, ".ppm") != NULL) {		//Only try to open .ppm files.
		   cout << "Reading ppm-file: " << dirname + entry->d_name <<endl;
		   if (!has_entry(&ppm_entries, remember)){	//Should never fail, but who knows...
			  ppm_entries.push_back(remember);  	//remember entry location			  
			  
		
			  current_entry = ppm_entries.size()-1;			  
		   }else  cout << "Unexpected error in CalibTool::nextImage. " 
			       << "--> Tried to remember an image I already knew..." << endl << flush;
             readImage((dirname + entry->d_name).c_str());	     
	     return true;
	   }
//	   else cout << "Skipping file: " << entry->d_name << endl;
	}
    }while (entry);
    cout << "No more images available." << endl << flush;
    return false;
   } else 	// go to next already read image.
   {
	   //TODO Exception handling
	   ++current_entry;	   
	   seekdir(hdir, ppm_entries.at(current_entry)); //set stream position to read to next image in list.
	   entry = readdir(hdir);			 //Get that image...
	   cout << "Reading (past) file: " << entry->d_name <<endl;
	   readImage((dirname+ entry->d_name).c_str());
	   return true;
   }
}

bool CalibTool::lastImage(){
	if (current_entry == 0){
		cout << "No previous image available..." << endl;
		return false;
	}
	current_entry -= 2;
	return nextImage();	
}


void CalibTool::readChessboard(const char* file_name){
    ifstream in(file_name);
    double x, y;
    in >> anz;
    points = new double*[anz];
    for (int i = 0 ; i < anz ; i++)
    	points[i] = new double[4];

    for (int i = 0; i < anz; i++){
        in >> x >> y;
        points[i][0] = x;
	points[i][1] = y;
    }
}

void CalibTool::init(int argc, char* argv[]){;
    cout 	<< "Usage:" << endl 
    		<< "--> Left-Click to add a point." << endl 
    		<< "--> Right-Click to remove last point." << endl
		<< "--> Hold Ctrl-Key to select point in zoomed view." << endl
		<< "--> Press enter to save points for current image and advance to next." << endl
		<< "--> Press Left- or Right-key to flip through images in the specified folder." << endl << endl << flush;
		
    //Initialize QApplication
    QApplication app(argc, argv);

    widget = new MyClickWidget(this, *image);
    widget->show();

    app.setMainWidget(widget);

    QObject::connect( &app, SIGNAL(lastWindowClosed()),
		      &app, SLOT(quit()) );

    while (app.mainWidget()->isVisible())
	app.processEvents();
  }

void
CalibTool::initChessboard(int cols, int rows, double cell_width, double cell_height){
  cb_rows=rows;
  cb_cols=cols;
  cb_width = cell_width;
  cb_height = cell_height;
}

void CalibTool::setPixel(int u, int v){
  if (pointc >= anz) {
  	cout << "Maximum number of points already specified. Press enter to save or right mouse button to remove last selection." << endl;
  	return;	
  }
  
  if(precision){ //sub-pixel precision triggered by Ctrl-Key
  	//calc displacement in scaled distance and add it to the orignial point
  	points[pointc][2] = ((u - orig_u) / SCALE) + orig_u;	//sub-pixel u
	points[pointc][3] = ((v - orig_v) / SCALE) + orig_v;	//sub-pixel v 
  } else { //normal click
  	points[pointc][2]=u;
  	points[pointc][3]=v;
  }
  cout << pointc + 1 <<": (" << points[pointc][0] << " " << points[pointc][1] << ") --> (" 
       		 	     << points[pointc][2] << " " << points[pointc][3] << ")" << flush << endl;
    ++pointc;
  
}

void CalibTool::removeLastPixel(){
  if (--pointc >= 0) 
     cout << "Point " << pointc + 1 << " removed." << endl;
  else {
     cout << "No point to remove. Press left mouse button to add point." << endl << flush;
     ++pointc;
  }
}

void CalibTool::zoom(int u, int v){
  if (image == NULL || originalImage == NULL) return;
  if (u >= image->getWidth()) u = image->getWidth()-1;
  if (v >= image->getHeight()) v = image->getHeight()-1;
  if (u < 0) u = 0;
  if (v < 0) v = 0;
  
  if (precision) return; //Move mouse in zoomed area --> Therefore, do not change zoom
  //else zoom area underneath mouse position
  
  //remember current original location.
  orig_u = u;
  orig_v = v;
  
  ImageBuffer::convert(originalImage->getImageBuffer(), 
		       image->getImageBuffer());

  //draw crosses to the already selected points
  markSelections();
  
  YUVTuple yuv;

  int i = u - U_DIM/2;
  int j = v - V_DIM/2;
  i = i < 0 ? 0 : i;
  j = j < 0 ? 0 : j;
  int org_j = j;
  int i_to = (u + U_DIM/2) > image->getWidth() ? image->getWidth() : u + U_DIM/2;
  int j_to = (v + V_DIM/2) > image->getHeight() ? image->getHeight() : v + V_DIM/2;

  int di, dj;
  for (; i < i_to; ++i){
  	for (; j<j_to; ++j){
	        di = i - u;
		dj = j - v;
		originalImage->getPixelYUV (u+di/SCALE, v+dj/SCALE, &yuv);
		image->setPixelYUV(i,j, yuv);
	}
	j = org_j;
  }
 //end zoom underneath mouse
}

void CalibTool::markSelections(){
   int delta = 5;
   YUVTuple yuv;
   yuv.y = 10;
   yuv.u = 10;
   yuv.v = 10;
   int i, j, neg_j;  
   for (int p = 0; p < pointc; p++){
   
      i = (int)points[p][2] - delta > 0 ? (int)points[p][2] - delta : 0;
      j = (int)points[p][3] - delta > 0 ? (int)points[p][3] - delta : 0;
      neg_j = (int)points[p][3] + delta < image->getHeight() ? (int)points[p][3] + delta : image->getHeight();
   
      //image is the fresh clone of the original image (see CalibTool::zoom(int, int)
      for (; i <= points[p][2] + delta && i < image->getWidth(); ++i){
          if (j < image->getHeight())
	     image->setPixelYUV(i, j++, yuv);
	  if (neg_j > 0)
	     image->setPixelYUV(i, neg_j--, yuv);
      }
      
   }
}

bool CalibTool::save(){
   if (pointc != anz){
      cout << "You have not selected the right amount of points "
	   << "specified in the pattern file:" << endl
           << "Selected: " << pointc << " Specified: " << anz << endl
	   << "File NOT saved." << endl << flush;
      return false;
   }
   ofstream o(output_file, ios::app );
   for (int i=0; i < pointc; ++i){	//Printing points that are saved.
     o << points[i][0] << " "  << points[i][1] << " " 
       << points[i][2] << " " << points[i][3] << " " << endl;
   }
   o << endl; //new line after one image.
   o.close();
   cout << pointc << " output points written to: " 
	<< output_file << endl << flush;
   if (hdir != NULL) {
      cout << "Opening next image." << endl;
      nextImage();
      pointc=0;
   }
   return true;
}

void CalibTool::usage(){
   cerr << endl << "Usage: ./CalibTool chessboard-metric-pattern coordinate-map ppm-image | -d directory" << endl 
   	<< endl 
        << "The chessboard-metric-pattern should contain: "  << endl
	<< " - The number of corners to mark in each image." << endl  
	<< " - The real world coordinates of the chessboard corners egocentrically" << endl
	<< "   from the robot in the form: x y (one pair per line)" << endl
	<< endl
	<< "The coordinate map is the resulting file in a format that can be used " << endl
	<< " as input to the DirectionalCalibration tool." << endl
	<< endl
	<< "You can process either a single image (only ppm is supported) or, " << endl
	<< " by setting parameter flag \"-d\", specify a directory with \".ppm\" files." << endl
	<< " All files that do not end on \".ppm\" in that directory will be skipped." << endl << endl << flush;
}

} //namspace Tribots
