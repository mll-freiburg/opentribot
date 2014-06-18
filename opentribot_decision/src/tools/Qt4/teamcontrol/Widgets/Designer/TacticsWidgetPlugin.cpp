
#include "../TacticsWidget.h"
#include "TacticsWidgetPlugin.h"

#include <QtCore/QtPlugin>

TacticsWidgetPlugin::TacticsWidgetPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *TacticsWidgetPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::TacticsWidget(parent);
}

QString TacticsWidgetPlugin::name() const {
  return "TribotsTools::TacticsWidget";
}

QString TacticsWidgetPlugin::group() const {
  return "Tribots Widgets";
}

QIcon TacticsWidgetPlugin::icon() const {
  return QIcon();
}

QString TacticsWidgetPlugin::toolTip() const {
  return "TacticsWidget";
}

QString TacticsWidgetPlugin::whatsThis() const {
  return "ein Widget zur Taktikauswahl";
}

bool TacticsWidgetPlugin::isContainer() const {
  return false;
}

QString TacticsWidgetPlugin::domXml() const {
  return "<widget class=\"TribotsTools::TacticsWidget\" name=\"TacticsWidget\">\n"
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

QString TacticsWidgetPlugin::includeFile() const {
  return "../TacticsWidget.h";
}

void TacticsWidgetPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool TacticsWidgetPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, TacticsWidgetPlugin)
