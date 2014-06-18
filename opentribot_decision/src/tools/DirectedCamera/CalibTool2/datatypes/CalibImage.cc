#include <iostream>
#include <fstream>
#include <vector>
#include "../../../../ImageProcessing/Formation/PPMIO.h"
#include "../../../../ImageProcessing/Formation/JPEGIO.h"
#include "../../../../ImageProcessing/Formation/RGBImage.h"
#include "../../../../ImageProcessing/Formation/ImageIO.h"
#include "CalibImage.h"
#include "../projectmanagement/ProjectManager.h"

CalibImage::CalibImage(QString imageURL, std::vector<ModelPoints> mPoints)
{
	
	// Dateinamen und Dateiendung extrahieren
	imageFileURL = imageURL;
	char point = '.';
	char slash = '/';
	int pospoint = imageFileURL.find ( point, 0, true );
	int posslash = imageFileURL.findRev ( slash );
	imageFileName = imageFileURL.remove(pospoint, imageFileURL.length()-1);
	imageFileName = imageFileURL.remove(0, posslash+1);
	imageFileURL = imageURL;
	imageFilePrefix = imageFileURL.remove(0, pospoint+1);
	
	// ModelPoints anlegen
	modelPoints = mPoints;	

	// Image anlegen
	if(imageFilePrefix.compare("jpeg") == 0 || imageFilePrefix.compare("jpg") == 0) {
		
		// JPEG behandeln
		
		std::ifstream in(imageURL);   	
   		Tribots::JPEGIO io;
   		tempImage =  new Tribots::RGBImage( *io.read(NULL, in) );
   		in.close();
   		originalImage = NULL;
	   	if (originalImage) {
	 		delete originalImage; 
	 	}
   		originalImage = tempImage->clone();
   		zoomImage = tempImage->clone();
		
	} else if(imageFilePrefix.compare("ppm") == 0) {
		
		// PPM behandeln
		
		std::ifstream in(imageURL.ascii()); 
   		Tribots::PPMIO io;
   		tempImage =  new Tribots::RGBImage( *io.read(NULL, in) );
   		in.close();
  		
   		originalImage = NULL;
	   	if (originalImage) {
	 		delete originalImage; 
	 	}
   		originalImage = tempImage->clone();
   		zoomImage = tempImage->clone();
   				
	} else {
		
		// throw exception
		
	}
}


CalibImage::~CalibImage() {  }

void CalibImage::refreshTempImage() 
{
	tempImage = originalImage->clone();	
}

Tribots::Image* CalibImage::getOriginalImage() 
{ 
	return originalImage; 
}

Tribots::Image* CalibImage::getTempImage() 
{ 
	return tempImage; 
}

Tribots::Image* CalibImage::getZoomImage() 
{ 
	return zoomImage; 
}

QString& CalibImage::getImageFileURL() 
{ 
	return imageFileURL; 
}

QString& CalibImage::getImageFileName() 
{ 
	return imageFileName; 
}

QString& CalibImage::getImageFilePrefix() 
{	
	return imageFilePrefix; 
}

std::vector<ModelPoints>& CalibImage::getModelPoints() 
{ 
	return modelPoints; 
}
