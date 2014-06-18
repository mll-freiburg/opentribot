#include <iostream>
#include <fstream>
#include <vector>
#include <qfile.h>
#include "HardDiscAccess.h"
#include "../datatypes/ModelPoints.h"
#include "../projectmanagement/ProjectManager.h"

void HardDiscAccess::parseProjectFile(QString projectFileURL, QString &projectDir, QString &name, int &picCount, QString &cpointsFileName, QString &mpointsFileName, std::vector<QString> &pics) 
{
	QFile file(projectFileURL);
	file.open(IO_ReadOnly);
	QTextStream stream(&file);
	QString line("");
	        
    bool ok;
        
    while (line != NULL) {
    	line = stream.readLine();	
        if (line.at(0) != '#') {
        		
        	// Projektnamen extrahieren
        	if (line.startsWith( "NAME=" )) {
        		int pos = line.find('=',0,true);
        		line.remove(0,pos+1);
        		name = line;        			
        	}
        	
        	if (line.startsWith( "MFILE=" )) {
        		int pos = line.find('=',0,true);
        		line.remove(0,pos+1);
        		mpointsFileName = line;
        	}
        	
        	if (line.startsWith( "CFILE=" )) {
        		int pos = line.find('=',0,true);
        		line.remove(0,pos+1);
        		cpointsFileName = line;
        	}
	        	
        	// Anzahl der Bilder extrahieren
        	if (line.startsWith( "PICTURECOUNT=" )) {
        		int pos = line.find('=',0,true);
        		line.remove(0,pos+1);
        		picCount = line.toInt( &ok, 10 );    
        		std::cerr << picCount;			
        	}		
	        		
        	// Bilder extrahieren
        	if (picCount > 0 && line.startsWith( "PICTURE=" )) {
        		int pos = line.find('=',0,true);
        		line.remove(0,pos+1);
        		QString pic = line;
        		pics.push_back(pic);		
        	}
        }     	
    }
	    
	    
    projectDir = projectFileURL;    
    int pos = projectDir.findRev('/');
    projectDir.remove(pos,projectDir.length());
	       
    file.close();
}


void HardDiscAccess::writeProjectFile(QString projectDir)
{
	// Projectnamen besorgen
	QString projectName = ProjectManager::getProject(ProjectManager::getActProjectID()).getProjectName();
	// modelPointsFileName besorgen
	QString mfile = ProjectManager::getProject(ProjectManager::getActProjectID()).getModelPointsFile();
	char slash = '/';
	int posslash = mfile.findRev ( slash );
	mfile = mfile.remove(0, posslash+1);
	// worldModelPointsFileName besorgen
	QString cfile = ProjectManager::getProject(ProjectManager::getActProjectID()).getWorldModelPointsFile();
	// Anzahl Bilder besorgen
	int count = ProjectManager::getProject(ProjectManager::getActProjectID()).getImageCount();
	
	// AusgabeStrom öffnen
	std::ofstream o;
	o.open(((projectDir.append("/")).append(projectName)).append(".capro"), std::ios::trunc);
	
	o << "# ATTENTION  ATTENTION  ATTENTION  ATTENTION  ATTENTION  ATTENTION" << std::endl;
	o << "# =========  =========  =========  =========  =========  =========" << std::endl;
	o << "#" << std::endl;
	o << "#	WARNING! All changes made in this file will maybe damage the project!" << std::endl;
	o << std::endl;
	o << "# Name des Projekts" << std::endl;
	o << "NAME=" << projectName << std::endl;
	o << std::endl;
	o << "# Name der Datei, die die Modellkoordinaten enthält" << std::endl;
	o << "MFILE=" << mfile << std::endl;
	o << std::endl;
	o << "# Name der Datei, die die Korrespondenzkoordinaten enthält" << std::endl;
	o << "CFILE=" << cfile << std::endl; 
	o << std::endl;
	o << "# Reihenfolge der Bilder beim letzten speichern" << std::endl;
	o << "# des Project, falls neue Bilder hinzukommen" << std::endl; 
	o << "# muessen diese angehaengt werden" << std::endl; 
	o << "PICTURECOUNT=" << count << std::endl;
	for(int i = 0; i < count; i++) {
		QString pictureString = (ProjectManager::getProject(ProjectManager::getActProjectID()).getCalibImage(i).getImageFileName());
		(pictureString.append(".")).append(ProjectManager::getProject(ProjectManager::getActProjectID()).getCalibImage(i).getImageFilePrefix());
		o << "PICTURE=" << pictureString  << std::endl;
		//o << "PICTURE=" << ((ProjectManager::getProject(ProjectManager::getActProjectID()).getCalibImage(i).getImageFileName()).append(".")).append(ProjectManager::getProject(ProjectManager::getActProjectID()).getCalibImage(i).getImageFilePrefix()) << std::endl; 	
	}
	o << std::endl;
	o << "# ATTENTION  ATTENTION  ATTENTION  ATTENTION  ATTENTION  ATTENTION" << std::endl;
	o << "# =========  =========  =========  =========  =========  =========" << std::endl;
	o << "#" << std::endl;
	o << "#	WARNING! All changes made in this file will maybe damage the project!" << std::endl;
	
	o.close();
}

std::vector<ModelPoints> HardDiscAccess::readModelPoints(QString mPointsFile)
{
	std::vector<ModelPoints> v;	
	double xm, ym;
	double xw = -1.0;
	double yw = -1.0;
    int anz;
		std::ifstream in(mPointsFile.ascii());
	    in >> anz;
	    for (int i = 0 ; i < anz ; i++) {
	    	in >> xm >> ym;
	    	v.push_back(ModelPoints(xm,ym,false,xw,yw));
	    } 	
	return v;
}

std::vector<ModelPoints> HardDiscAccess::readWorldPoints(QString cPointsFile)
{
	// Bilder müssen einzeln behandelt werden!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	std::vector<ModelPoints> v;	
	double xm, ym, xw, yw;
    int breite,hoehe,anzbilder,anzpunkte;
		std::ifstream in(cPointsFile.ascii());
	    in >> breite;
	    in >>  hoehe;
	    in >> anzbilder;
	    in >> anzpunkte;
	    for(int i = 0; i < anzbilder*anzpunkte; i++) {
		    in >> xm; 
		    in >> ym; 
		    in >> xw; 
		    in >> yw;
		    if(xw == -1 && yw == -1) {
		    	v.push_back(ModelPoints(xm,ym,false,xw,yw));
		    } else if (xw != -1 && yw != -1) {
		    	v.push_back(ModelPoints(xm,ym,true,xw,yw));
		    }	    	
		} 	
	return v;
}

void HardDiscAccess::writeModelWorldPointsFile(QString cPointsFile)
{
	ProjectManager::getProject(ProjectManager::getActProjectID()).setWorldModelPointsFile("cpoints.txt");
	
	// AusgabeStrom öffnen
	std::ofstream o;
	o.open(cPointsFile.append("/cpoints.txt"), std::ios::trunc);
	
	int proID = ProjectManager::getActProjectID();
	
	// Anzahl Bilder
	int imageCount = ProjectManager::getProject(proID).getImageCount();
	
	// Anzahl der Punkte
	int pointCount = (((ProjectManager::getProject(proID)).getCalibImage(0)).getModelPoints()).size();
	// Breite der Bilder
	int width = (((ProjectManager::getProject(proID)).getCalibImage(0)).getOriginalImage())->getWidth();
	// Hoehe der Bilder
	int height = (((ProjectManager::getProject(proID)).getCalibImage(0)).getOriginalImage())->getHeight();;
	
	o << width << " " << height << " " << imageCount << " " << pointCount << std::endl;
	o << std::endl;
	for(int i = 0; i < imageCount; i++) {
		for( unsigned int j = 0; j < pointCount; j++) {
			o << (((ProjectManager::getProject(proID)).getCalibImage(i)).getModelPoints()).at(j).getXModel() << " ";	
			o << (((ProjectManager::getProject(proID)).getCalibImage(i)).getModelPoints()).at(j).getYModel() << " ";
			o << (((ProjectManager::getProject(proID)).getCalibImage(i)).getModelPoints()).at(j).getXWorld() << " ";
			o << (((ProjectManager::getProject(proID)).getCalibImage(i)).getModelPoints()).at(j).getYWorld() << std::endl;
		}	
		o << std::endl; // neue Zeile nach einem Image
	}	
	o.close();
}