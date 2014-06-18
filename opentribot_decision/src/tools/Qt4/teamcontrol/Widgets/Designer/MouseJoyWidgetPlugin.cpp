
#include "../MouseJoyWidget.h"
#include "MouseJoyWidgetPlugin.h"

#include <QtCore/QtPlugin>

MouseJoyWidgetPlugin::MouseJoyWidgetPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *MouseJoyWidgetPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::MouseJoyWidget(parent);
}

QString MouseJoyWidgetPlugin::name() const {
  return "TribotsTools::MouseJoyWidget";
}

QString MouseJoyWidgetPlugin::group() const {
  return "Tribots Widgets";
}

QIcon MouseJoyWidgetPlugin::icon() const {
  return QIcon();
}

QString MouseJoyWidgetPlugin::toolTip() const {
  return "MouseJoyWidget";
}

QString MouseJoyWidgetPlugin::whatsThis() const {
  return "ein Widget zur Simulation eines Joysticks per Maus";
}

bool MouseJoyWidgetPlugin::isContainer() const {
  return false;
}

QString MouseJoyWidgetPlugin::domXml() const {
  return "<widget class=\"TribotsTools::MouseJoyWidget\" name=\"MouseJoyWidget\">\n"
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

QString MouseJoyWidgetPlugin::includeFile() const {
  return "../MouseJoyWidget.h";
}

void MouseJoyWidgetPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool MouseJoyWidgetPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, MouseJoyWidgetPlugin)
