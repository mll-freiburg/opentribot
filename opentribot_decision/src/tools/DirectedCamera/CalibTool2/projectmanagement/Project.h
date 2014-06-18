#ifndef _PROJECT_H_
#define _PROJECT_H_

#include <vector>
#include <qstring.h>
#include <qfile.h>
#include "../../../../ImageProcessing/Formation/Image.h"
#include "../datatypes/ModelPoints.h"
#include "../datatypes/Points.h"
#include "../datatypes/CalibImage.h"

/**
 * Aufgabe: Repraesentiert ein internes Project. Speichert alle relevanten Daten 
 * Verfuegt ueber eine Projekt-ID zur Handhabung
 */
class Project {

 	public:	 		
 		/** Standardkonstruktor -> wird benoetigt von QMap */
 		Project();
  		
  		/** legt ein neues Project an mit hilfe der uebergebenen Variablen */	  		
  		Project(QString proName, QString projectDir, bool newProject);
  		
  		virtual ~Project();  		
  		
  		/** fuegt dem Project ein Bild hinzu  */
  		void addPicture(QString imagePath);  		
  		
  		/** fuegt dem Project ModelKoordinaten hinzu */
  		void addModelPoints(ModelPoints*);  
  				
  		/** liefert den Namen des Projects */
  		QString& getProjectName();  
  				
  		/** liefert den Pfad des Projects */
  		QString& getProjectPath();  	
  			
  		/** setzt den Namen des Projects */
  		void setProjectName(QString projectName);
  		
  		/** setzt den Pfad des Projects */
  		void setProjectPath(QString projectURL);
  		
  		/** liefert die Project ID */
  		int getProjectID();
  		
  		/** setzt die Project ID */
  		void setProjectID(int id);
  		
  		/**  */
  		void setWorldModelPointsFile(QString name);
  		
  		/**  */
  		void setModelPointsFile(QString name);
  		
  		/**  */
  		QString& getModelPointsFile();
		
		/**  */
		QString& getWorldModelPointsFile();
		
		/**  */
		int getImageCount();
		
		/**  */
		std::vector<CalibImage>& getCalibImages();
		
		/** liefert das bestimmte Bild an Stelle n im Array */
		CalibImage& getCalibImage(int n);
		
		
		
		

	private:
		/** Project Name */
  		QString projectName;
  		
  		/** Project URL */
  		QString projectPath;
  		
		/** ModelPointsFile */
		QString modelPointsFile;
		
		/** WorldModelPointsFile */
		QString worldModelPointsFile;
		
		/** Anzahl der Bilder im Project */
		int imageCount;
		
		/** enthaelt alle Bilder des Projects (Array) */
		std::vector<CalibImage> calibImages;
   
};

#endif //_PROJECT_H_

