
#include "../CameraSettingsWidget.h"
#include "CameraSettingsWidgetPlugin.h"

#include <QtCore/QtPlugin>

CameraSettingsWidgetPlugin::CameraSettingsWidgetPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *CameraSettingsWidgetPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::CameraSettingsWidget(parent);
}

QString CameraSettingsWidgetPlugin::name() const {
  return "TribotsTools::CameraSettingsWidget";
}

QString CameraSettingsWidgetPlugin::group() const {
  return "Tribots Widgets";
}

QIcon CameraSettingsWidgetPlugin::icon() const {
  return QIcon();
}

QString CameraSettingsWidgetPlugin::toolTip() const {
  return "CameraSettingsWidget";
}

QString CameraSettingsWidgetPlugin::whatsThis() const {
  return "ein Widget, um Kamera-Features anzuzeigen und zu veraendern";
}

bool CameraSettingsWidgetPlugin::isContainer() const {
  return false;
}

QString CameraSettingsWidgetPlugin::domXml() const {
  return "<widget class=\"TribotsTools::CameraSettingsWidget\" name=\"CameraSettingsWidget\">\n"
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

QString CameraSettingsWidgetPlugin::includeFile() const {
  return "../CameraSettingsWidget.h";
}

void CameraSettingsWidgetPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool CameraSettingsWidgetPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, CameraSettingsWidgetPlugin)
