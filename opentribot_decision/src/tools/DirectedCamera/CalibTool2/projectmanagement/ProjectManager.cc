#include <qstring.h>
#include <qmap.h>
#include "ProjectManager.h"
#include "Project.h"


/** zaehlt die offenen Projects */
int ProjectManager::projectCounter = 0;
/** aktuelle Project ID */
int ProjectManager::actProjectID = 0;
/** speichert alle offenen Projects in der Map */
QMap<int,Project> ProjectManager::projectMap;


/** neues Project wird angelegt und in die QMap gepackt  */
void ProjectManager::newProject(QString proName, QString projectDir)
{
	Project newProject(proName, projectDir, true);
	projectMap.insert(++projectCounter,newProject,false);
	// aktuelle ProjectID festlegen
	ProjectManager::setActProjectID(projectCounter);
}

/** oeffnet ein bestehendes Projectund packt es in QMap */
void ProjectManager::openProject(QString proName, QString projectDir)
{
	Project newProject(proName, projectDir,false);
	projectMap.insert(++projectCounter,newProject,false);
	// aktuelle ProjectID festlegen
	ProjectManager::setActProjectID(projectCounter);
}

/** Project wird aus der Map entfernt */
void ProjectManager::closeProject(int id)
{
	if(id == projectMap.size()) {
		projectMap.remove(projectMap.find(id));
		ProjectManager::setActProjectID(--projectCounter);
	} else {
		int i = 0;
		for(i = id; i < projectMap.size(); i++) {
			//Project temp = projectMap[i+1];
			projectMap.replace(i,projectMap[i+1]);				
		}
		std::cout << "letzes Item an Position" << i+1 << std::endl;
		projectMap.remove(projectMap.find(projectMap.size()));
		projectCounter--;
	}
	
	
//	if(id == projectMap.size()) {
//		ProjectManager::setActProjectID(projectCounter-1);
//	}
//	if(id < projectMap.size()) {
//		ProjectManager::setActProjectID(projectCounter);
//	}
//	projectMap.remove(projectMap.find(id));
//	projectCounter--;
//	if(id > 0) {
//		ProjectManager::setActProjectID(--projectCounter);
//	}
//	std::cout << "projectCounter= " << projectCounter << std::endl; 
}

/** Project mit id=x wird gesucht und eine Referenz zurueckgegeben */		
Project& ProjectManager::getProject(int id)
{	
	return (projectMap.find(id)).data();
}

/**  */
int ProjectManager::getActProjectID() { return actProjectID; }

/**  */
void ProjectManager::setActProjectID(int id) { actProjectID = id; }

/**  */
int ProjectManager::getProjectCounter() { return projectCounter; }
	