#ifndef _calib_tool_h_
#define _calib_tool_h_

#include <iostream>
#include <fstream>
#include <dirent.h> //to read whole directories
#include <vector>

#include "../../../tools/components/ImageWidget.h"
#include "../../../ImageProcessing/Formation/Image.h"
#include "../../../ImageProcessing/Formation/YUVImage.h"
#include "../../../ImageProcessing/Formation/ImageIO.h"


using namespace std;

namespace Tribots{

  class MyClickWidget; //Implementation and comment see: CalibTool.cc

/** 
 *   This class provides methods to select points in the format needed for the 
 *   DirectionalCalibration tool.
 *
 *   Call:
 *	./CalibTool chessboard-metric-pattern coordinate-map ppm-image | -d directory
 *	
 *	The chessboard-metric-pattern should contain:
 *	 - The number of corners to mark in each image.
 *	 - The real world coordinates of the chessboard corners egocentrically
 *	    from the robot in the form: x y (one pair per line)
 *  
 *	The coordinate map is the resulting file in a format that can be used 
 *	 as input to the DirectionalCalibration tool.
 
 *	You can process either a single image (only ppm is supported) or, 
 *	 by setting parameter flag -d, specify a directory with .ppm files.
 *	 All files that do not end on .ppm in that directory will be skipped automatically.
 *
 *   Usage:
 *	--> Left-Click to add a point.
 *	--> Right-Click to remove last point.
 *	--> Hold Ctrl-Key to select point in zoomed view.
 *	--> Press enter to save points for current image and advance to next.
 *	--> Press Left- or Right-key to flip through images in the specified folder.
 *
 *   Authors: Christopher Loerken, cloerken@uos.de
 *            Tobias Kringe, tkringe@uos.de
 * 
 *   2005-08-09 Repaired several memory leaks, Sascha Lange, salange@uos.de, 
 *
 *   Date: 16.06.2005
 */
class CalibTool{

 protected:


  MyClickWidget* widget;

  int cb_cols, cb_rows, pointc, anz;
  Image* image;
  Image* originalImage;
  double cb_width, cb_height;
  double** points; 			//Contains 4 entries: [world_x, world_y, picture_u, picture_v]
  const char* output_file;
  bool precision;
  double orig_u, orig_v; 		//Remember for subpixel precision.
  DIR *hdir;				//If directory of images is to be read.
  vector<long> ppm_entries; 		//Stores the entries of already found ppm files in that directory. Used for flipping through.
  int current_entry;	    		//Location in the ppm_entries- vector of currently read image.

  string dirname;

 public:


  /**Constructor. Needs argmuents from main. argv has to have the following format:
   * argv[0] = normal argv[0];
   * argv[1] = metric pattern of chessboard (first entry has to be the number of points
   *	       and then x y real world coordinates of the corners.
   *	       Take care of the right ordering (has to be the same as your click order.))
   * argv[2] = output file. The metric and pixelcoordinates are appended at the end of the file.
   * argv[3] = ppm picture of the chessboard for the extrinsic calibration. Check consistency to the
   * 	       real world coordinates or '-d directory' if a whole directory shall be loaded.
   */
  CalibTool(int argc, char* argv[]);
  ~CalibTool();

  //------------------ ImageIO -------------------------------------------------------------//
  
  /** Reads Image to show it in the QApllication */
  void readImage(const char* image_name);
  
  /** Gets next iamge from Directory or from the list of remembered files (ppm_entries). */
  bool nextImage();
  
  /** Gets last image from the list of remembered files (ppm_entries). */
  bool lastImage();
  
  //----------------- Support functions for ClickWidget ------------------------------------//
  
  /** Setter for current_entry. Called by the forward-backward actions in the ClickWidget. */
  //void setCurrentEntry(int new_current) {  current_entry = new_current; }
  
  /** Getter for current_entry. Called by the forward-backward actions in the ClickWidget. */
  //int getCurrentEntry() {  return current_entry; }
  
  /** Getter for the image that is currently shown. Called each time that an action in the 
    * widget has been performed. */
  Image* getImage(){ return image; }
  
  
  //----------------- Chessboard properties and main functionality -------------------------//
   /**Reads the chessboard pattern file.*/
   void readChessboard(const char* file_name);

  /** Sets parameters for the chessboard, which is used.
    *  Important for retrieving real world coordinates; */
  void initChessboard(int cols=7, int rows=7, double cell_width=1.0, double cell_height=1.0);
   
  /** Stores the u,v pixel coordinates as the corresponding pixels to the next
   *  corner point. */
   void setPixel(int u, int v);
   void removeLastPixel();

   /**Zooms into the image at the given position*/
   void zoom(int u, int v);
   
   /**In the high precision mode the mouse can be moved within the zoomed area to achieve
    * sub-pixel accuracy iff true. */
   inline void setHighPrecision(bool b) {precision = b;}
   
   /** Draws a cross to the points that have already been selected.
    *  Attention: The crosses are at the approximate (not sub-pixel correct) position.
    *		  They are meant solely as a help for the user showing which location
    *		  has already been selected. */
   void markSelections();

   /** Initializes the QApllication and prints the usage string. */
   void init(int argc, char* argv[]);
   
   /**Saves the current points** list to the result file specified in the arguments.*/ 
   bool save();
   
   /**Prints how to call the program.*/
   static void usage();
};// end CalibTool



} // end namespace Tribots
#endif
