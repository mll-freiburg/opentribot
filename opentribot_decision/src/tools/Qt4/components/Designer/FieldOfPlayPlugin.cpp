
#include "../FieldOfPlay.h"
#include "FieldOfPlayPlugin.h"

#include <QtCore/QtPlugin>

FieldOfPlayPlugin::FieldOfPlayPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *FieldOfPlayPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::FieldOfPlay(parent);
}

QString FieldOfPlayPlugin::name() const {
  return "TribotsTools::FieldOfPlay";
}

QString FieldOfPlayPlugin::group() const {
  return "Tribots Widgets";
}

QIcon FieldOfPlayPlugin::icon() const {
  return QIcon();
}

QString FieldOfPlayPlugin::toolTip() const {
  return "FieldOfPlay";
}

QString FieldOfPlayPlugin::whatsThis() const {
  return "ein Widget, um ein MSL-Spielfeld darzustellen";
}

bool FieldOfPlayPlugin::isContainer() const {
  return false;
}

QString FieldOfPlayPlugin::domXml() const {
  return "<widget class=\"TribotsTools::FieldOfPlay\" name=\"FieldOfPlay\">\n"
            " <property name=\"geometry\">\n"
            "  <rect>\n"
            "   <x>0</x>\n"
            "   <y>0</y>\n"
            "   <width>300</width>\n"
            "   <height>200</height>\n"
            "  </rect>\n"
            " </property>\n"
            "</widget>\n";
}

QString FieldOfPlayPlugin::includeFile() const {
  return "../FieldOfPlay.h";
}

void FieldOfPlayPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool FieldOfPlayPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, FieldOfPlayPlugin)
