/*
 * Description: Implementation of Project.
 * Created:     2006-04-12
 * Author:      Jan Schädlich
 * Mail:        mail@janschaedlich.de
 * --------------------------------------------------------------------
 */

#include <iostream>
#include <qstring.h>
#include <qdir.h>
#include <qtextstream.h>
#include <qmessagebox.h>
#include "Project.h"
#include "HardDiscAccess.h"
#include "../datatypes/CalibImage.h"

using namespace std;

Project::Project()
{
	
}
  		
/** legt ein neues Project an mit hilfe der uebergebenen Variablen */	  		
Project::Project(QString proName, QString projectDir, bool newProject)
{	
	// Name des Projects
	projectName = proName;
		
	// Pfad zum ProjectOrdner
	projectPath = projectDir;
	
	
	// ein nagelneues Project wird angelegt
	if(newProject) {
		
		// Ordner nach txt Datei durchsuchen
		QDir txtDir(projectDir, "*.txt", QDir::Name | QDir::IgnoreCase , QDir::All);
		if(txtDir.count() == 0) {		
			std::cout << "keine txt Datei vorhanden!";
		} else if(txtDir.count() > 1) {
			//std::cout << "zuviele txt Dateien vorhanden!" << endl;
		} else {
			QString tempDir = projectDir;
			modelPointsFile = tempDir.append("/"+txtDir.operator[](0));
		} 
		
		// Ordner nach Bildern  durchsuchen	
		// und CalibImages anlegen
		QDir picDir(projectDir, "*.ppm *.jpeg *.jpg", QDir::Name | QDir::IgnoreCase , QDir::All);
		if(picDir.count() == 0) {
			std::cout << "keine Bilder vorhanden!";
		} else {
			QString tempDir = projectDir;
			imageCount = picDir.count();
			
			for(unsigned int i = 0; i < picDir.count(); i++ ) {
				QString temp = picDir.operator[](i);
				CalibImage image(tempDir.append("/"+picDir.operator[](i)), HardDiscAccess::readModelPoints(modelPointsFile));
				tempDir = projectDir;
				calibImages.push_back(image);
			}
		}	
	}
	
	// ein bestehendes Project wird angelegt
	else if(!newProject) {
		
		QString name,cpointsFileName,mpointsFileName,proDir;
	    int picCount;
	    std::vector<QString> pics;
	    
	    QString path = projectDir;
	    path = path.append("/").append(proName).append(".capro");
	    
        HardDiscAccess::parseProjectFile(path, proDir, name, picCount, cpointsFileName, mpointsFileName, pics);
		
		QString tempDir = projectDir;
		worldModelPointsFile = tempDir.append("/").append(cpointsFileName);
		tempDir = projectDir;
		modelPointsFile = tempDir.append("/").append(mpointsFileName);
		
		// und CalibImage anlegen

		tempDir = projectDir;
		imageCount = picCount;
		std::vector<ModelPoints> v;
		v = HardDiscAccess::readWorldPoints(worldModelPointsFile);
		int pointCount = v.size()/imageCount;
		for(unsigned int i = 0; i < imageCount; i++ ) {
			std::vector<ModelPoints> vn;
			for(unsigned int j = i*pointCount; j < (i*pointCount)+pointCount; j++) {
				vn.push_back(v.at(j));						
			}
			CalibImage caImage(tempDir.append("/"+pics.at(i)), vn);
			tempDir = projectDir;
			calibImages.push_back(caImage);
			
		}		
	}
}
		  		
/**  */	
Project::~Project() 
{
		
}

/**  */	
void Project::addPicture(QString imagePath) 
{
	std::cout << imagePath << std::endl;
	std::cout << modelPointsFile << std::endl;
	CalibImage temp(imagePath, HardDiscAccess::readModelPoints(modelPointsFile));
	calibImages.push_back(temp);
	imageCount++;
}

/**  */	
void Project::addModelPoints(ModelPoints*) {  }

std::vector<CalibImage>& Project::getCalibImages() { return calibImages; }

/**  */
CalibImage& Project::getCalibImage(int n)
{
	if(calibImages.size() == 0) {
		std::cerr << "vector leer";
	}
	return calibImages.at(n);	
}

/**  */		
QString& Project::getProjectName() { return projectName; }

/**  */	
QString& Project::getProjectPath() { return projectPath; } 
 		
/**  */	
void Project::setProjectName(QString projectName) { projectName = projectName; }

/**  */	
void Project::setProjectPath(QString projectURL) {	projectPath = projectURL; }

/**  */
void Project::setWorldModelPointsFile(QString name) { worldModelPointsFile = name; }

/**  */
void Project::setModelPointsFile(QString name) { modelPointsFile = name; }

/**  */
int Project::getImageCount() { return imageCount; }

/**  */
QString& Project::getModelPointsFile() { return modelPointsFile; }

/**  */
QString& Project::getWorldModelPointsFile() { return worldModelPointsFile; } 
