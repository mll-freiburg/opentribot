
#include "../MonoLED.h"
#include "MonoLEDPlugin.h"

#include <QtCore/QtPlugin>

MonoLEDPlugin::MonoLEDPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *MonoLEDPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::MonoLED(parent);
}

QString MonoLEDPlugin::name() const {
  return "TribotsTools::MonoLED";
}

QString MonoLEDPlugin::group() const {
  return "Tribots Widgets";
}

QIcon MonoLEDPlugin::icon() const {
  return QIcon();
}

QString MonoLEDPlugin::toolTip() const {
  return "MonoLED";
}

QString MonoLEDPlugin::whatsThis() const {
  return "ein Widget, um ein die orientierung eines Roboters anzuzeigen";
}

bool MonoLEDPlugin::isContainer() const {
  return false;
}

QString MonoLEDPlugin::domXml() const {
  return "<widget class=\"TribotsTools::MonoLED\" name=\"MonoLED\">\n"
            " <property name=\"geometry\">\n"
            "  <rect>\n"
            "   <x>0</x>\n"
            "   <y>0</y>\n"
            "   <width>40</width>\n"
            "   <height>40</height>\n"
            "  </rect>\n"
            " </property>\n"
            "</widget>\n";
}

QString MonoLEDPlugin::includeFile() const {
  return "../MonoLED.h";
}

void MonoLEDPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool MonoLEDPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, MonoLEDPlugin)
