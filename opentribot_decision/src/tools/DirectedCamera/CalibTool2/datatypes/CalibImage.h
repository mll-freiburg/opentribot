#ifndef CALIBIMAGE_H_
#define CALIBIMAGE_H_

#include <vector>
#include <qstring.h>
#include "ModelPoints.h"
#include "../../../../ImageProcessing/Formation/Image.h"

/** 
 * Klasse, die ein Bild und die dazugehörigen ModelPoints enthält
 *  */
class CalibImage 
{

	public:
		/** 
		 * Erzeugt ein neues CalibImage.
		 * 
		 * \param imageURL  kompletter Pfad zum Bild
		 * \parem mPoints   vector der die ModelPoints enthält
		 */
		CalibImage(QString imageURL, std::vector<ModelPoints> mPoints);
		
		virtual ~CalibImage();
		
		/** Gibt das Original-Image zurück. Auf diesem befinden sich keine Markierungen! */
		Tribots::Image* getOriginalImage();
		
		/** Gibt das Temp-Image zurück. Auf diesem befinden sich die Markierungen! */
		Tribots::Image* getTempImage();
		
		/** Gibt das Zoom-Image zurück. Auf diesem befindet sich der zom Ausschnitt! */
		Tribots::Image* getZoomImage();
		
		/** Das Temp-Image wird neu gesetzt. */
		void refreshTempImage();
		
		
		/** Liefert den kompletten Pfad des Original-Bildes zurück */
		QString& getImageFileURL();
		
		/** Liefert den Dateinamen des Original-Bildes mit Prefix zurück. */
		QString& getImageFileName();
		
		/** Liefert den Datei-Prefix des Original-Bildes zurück. */
		QString& getImageFilePrefix();

		/** Liefert denvector mit ModelPoints zurück. */
		std::vector<ModelPoints>& getModelPoints();
		
	private:
		/** Originalbild. */
		Tribots::Image* originalImage;
		
		/** Bild auf dem gezeichnet wird. */
		Tribots::Image* tempImage;
		
		/** Bild mit zoom Ausschnitt. */
		Tribots::Image* zoomImage;
		
		/** Kompletter Pfad zum Bild. */
		QString imageFileURL;
		
		/** Dateiname des Bildes ohne Endung. */
		QString imageFileName;
		
		/** Endung der Bilddatei (jpg, ppm, ...) */
		QString imageFilePrefix;
  		
  		/** ModelKoordinaten mit Wert, ob schon geklickt oder nicht (Array) */
  		std::vector<ModelPoints> modelPoints;
};

#endif /*CALIBIMAGE_H_*/
