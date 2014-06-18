
#include "../ValPlotWidget.h"
#include "ValPlotWidgetPlugin.h"

#include <QtCore/QtPlugin>

ValPlotWidgetPlugin::ValPlotWidgetPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *ValPlotWidgetPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::ValPlotWidget(parent);
}

QString ValPlotWidgetPlugin::name() const {
  return "TribotsTools::ValPlotWidget";
}

QString ValPlotWidgetPlugin::group() const {
  return "Tribots Widgets";
}

QIcon ValPlotWidgetPlugin::icon() const {
  return QIcon();
}

QString ValPlotWidgetPlugin::toolTip() const {
  return "ValPlotWidget";
}

QString ValPlotWidgetPlugin::whatsThis() const {
  return "ein Widget, um Werte fortlaufend zu plotten";
}

bool ValPlotWidgetPlugin::isContainer() const {
  return false;
}

QString ValPlotWidgetPlugin::domXml() const {
  return "<widget class=\"TribotsTools::ValPlotWidget\" name=\"ValPlotWidget\">\n"
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

QString ValPlotWidgetPlugin::includeFile() const {
  return "../ValPlotWidget.h";
}

void ValPlotWidgetPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool ValPlotWidgetPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, ValPlotWidgetPlugin)
