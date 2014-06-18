#ifndef HARDDISCACCESS_H_
#define HARDDISCACCESS_H_

#include <vector>
#include <qstring.h>
#include "../datatypes/ModelPoints.h"
#include "../datatypes/Points.h"

/** 
 * Aufgabe: Handhabung aller vom Programm benoetigten Festplattenzugriffe 
 * Auf Grund der allgemeinen Aufgabe sind alle Methoden statisch 
 */
class HardDiscAccess
{
	public:
		/** Liest das Project-File ein und parsed dessen Inhalt! */
		static void parseProjectFile(QString projectFileURL, QString &projectDir, QString &name, int &picCount, QString &cpointsFileName, QString &mpointsFileName, std::vector<QString> &pics);
		
		/** Schreibt ein Project-File. */
		static void writeProjectFile(QString projectDir);
		
		/** Liest eine ModelPoints-Datei ein und liefert einen vector mit Punkten zurück. */
		static std::vector<ModelPoints> readModelPoints(QString mPointsFile);
		
		/** Liest eine WorldPoints-Datei ein und liefert einen vector mit Punkten zurück. */
		static std::vector<ModelPoints> readWorldPoints(QString cPointsFile);
		
		/** Schreibt die Korrespondenz-Punkte-Datei. */
		static void writeModelWorldPointsFile(QString cPointsFile);
};

#endif /*HARDDISCACCESS_H_*/
