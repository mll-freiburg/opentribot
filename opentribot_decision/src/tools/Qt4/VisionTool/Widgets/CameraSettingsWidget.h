
#ifndef _TribotsTools_CameraSettingsWidget_h_
#define _TribotsTools_CameraSettingsWidget_h_

#include <QtGui/QScrollArea>
#include <QtGui/QFrame>
#include <QtCore/QTimer>
#include <iostream>
#include "../../../../ImageProcessing/Formation/IIDC.h"


namespace TribotsTools {

  /** Widget, um Kameraeinstellungen vorzunehmen (Shutter, Gain, Whitebalance, etc.) */
 class CameraSettingsWidget : public QScrollArea {
   Q_OBJECT

 public:
   /** Konstruktor, anschliessend muss init (.) aufgerufen werden */
   CameraSettingsWidget ( QWidget* =0, Qt::WFlags = 0 );
   ~CameraSettingsWidget ();

   /** Initialisieren des Fensters, Uebergabe der Kamera, arg2=MemoryChannel anzeigen, wenn vorhanden? */
   void init (Tribots::IIDC*, bool memChan, bool head);
   /** Loeschen des Fensterinhalts */
   void deinit ();

   /** Schreiben der Kamera-Feature-Werte in Configdatei */
   void writeFeatureSettings (std::ostream&);
   /** Schreiben der Kamera-Feature-Werte in ConfigReader, arg2=Sectionname */
   void writeFeatureSettings (Tribots::ConfigReader&, const std::string&);

 signals:
   void widgetClosed ();   ///< is called when the widget is hidden

 protected slots:
   /** aktualisiert alle angezeigten Werte und alle ComboBoxen */
   void showEvent ( QShowEvent * );
   /** ausschalten des Timer */
   void hideEvent ( QHideEvent * );
   /** die Auto-Werte richtig anzeigen */
   void updateAutoFeatures ();
   /** die Anzeige des (arg1)-ten Feature aktualisieren */
   void updateFeatureValue (unsigned int);
   /** die Anzeige des (arg1)-ten Featuremodus aktualisieren */
   void updateFeatureMode (unsigned int);
   /** ein Slider mit ID (arg1) hat sich veraendert auf den Wert (arg2) */
   void sliderChanged (unsigned int, unsigned int);
   /** eine SpinBox mit ID (arg1) hat sich veraendert auf den Wert (arg2) */
   void spinBoxChanged (unsigned int, unsigned int);
   /** eine ComboBox mit ID (arg1) hat sich veraendert auf den Wert (arg2) */
   void comboBoxChanged (unsigned int, const QString&);
   /** Lade aus Memory Channel */
   void loadMemoryChannel ();
   /** Speichere in Memory Channel */
   void saveMemoryChannel ();

 protected:
   struct FeatureInfo;
   FeatureInfo* features;      ///< fuer jedes Feature eine beschreibendes Strukt
   unsigned int num_features;

   Tribots::IIDC* camera;

   bool lockedCamera;  // false: Kameraeinstellungen sollen geaendert werden, true: nur Anzeige soll geaendert werden
   bool lockedDisplay;  // false: Anzeige soll geaendert werden, true: Anzeige darf nicht geaendert werden

   QTimer* timer;
 };

}

#endif
