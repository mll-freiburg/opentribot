
#include "../DoubleSlider.h"
#include "DoubleSliderPlugin.h"

#include <QtCore/QtPlugin>

DoubleSliderPlugin::DoubleSliderPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *DoubleSliderPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::DoubleSlider(parent);
}

QString DoubleSliderPlugin::name() const {
  return "TribotsTools::DoubleSlider";
}

QString DoubleSliderPlugin::group() const {
  return "Tribots Widgets";
}

QIcon DoubleSliderPlugin::icon() const {
  return QIcon();
}

QString DoubleSliderPlugin::toolTip() const {
  return "DoubleSlider";
}

QString DoubleSliderPlugin::whatsThis() const {
  return "ein Slider fuer Intervalle";
}

bool DoubleSliderPlugin::isContainer() const {
  return false;
}

QString DoubleSliderPlugin::domXml() const {
  return "<widget class=\"TribotsTools::DoubleSlider\" name=\"DoubleSlider\">\n"
            " <property name=\"geometry\">\n"
            "  <rect>\n"
            "   <x>0</x>\n"
            "   <y>0</y>\n"
            "   <width>200</width>\n"
            "   <height>40</height>\n"
            "  </rect>\n"
            " </property>\n"
            "</widget>\n";
}

QString DoubleSliderPlugin::includeFile() const {
  return "../DoubleSlider.h";
}

void DoubleSliderPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool DoubleSliderPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, DoubleSliderPlugin)
