#ifndef _PROJECTMANAGER_H_
#define _PROJECTMANAGER_H_

#include <qstring.h>
#include <qmap.h>
#include <vector>
#include "Project.h"

/** 
 * Aufgabe: Verwaltet die intern benoetigten Projects in einer Map
 * Die Funktionalitaet wird allgemein benoetigt, 
 * deshalb enthaelt diese Klasse nur statische Methoden
 */
class ProjectManager {

	public:
		/** legt ein neues Project an */
		static void newProject(QString proName, QString projectDir);
		
		/** oeffnet ein bestehendes Project */
		static void openProject(QString proName, QString projectDir);	
				
		/** schliesst ein offenes Project */
		static void closeProject(int id);	
			
		/** liefert Project mit id=id */		
		static Project& getProject(int id);
		
		/** liefert die aktuelle Project ID zurueck */
		static int getActProjectID();
		
		/** setzt die aktuelle Project ID */
		static void setActProjectID(int id);
		
		/** liefert den aktuellen Projectzaehler zurueck */
		static int getProjectCounter();
	
	private:		
		/** speichert alle offenen Projects in der Map */
		static QMap<int,Project> projectMap;	
				
		/** zaehlt die offenen Projects */
  		static int projectCounter;
  		
  		/** aktuelle Project ID */
  		static int actProjectID;
};

#endif //_PROJECTMANAGER_H_
