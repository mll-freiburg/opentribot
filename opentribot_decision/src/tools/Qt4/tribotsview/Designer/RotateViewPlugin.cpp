
#include "../RotateView.h"
#include "RotateViewPlugin.h"

#include <QtCore/QtPlugin>

RotateViewPlugin::RotateViewPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *RotateViewPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::RotateView(parent);
}

QString RotateViewPlugin::name() const {
  return "TribotsTools::RotateView";
}

QString RotateViewPlugin::group() const {
  return "Tribots Widgets";
}

QIcon RotateViewPlugin::icon() const {
  return QIcon();
}

QString RotateViewPlugin::toolTip() const {
  return "RotateView";
}

QString RotateViewPlugin::whatsThis() const {
  return "ein Widget, um ein die orientierung eines Roboters anzuzeigen";
}

bool RotateViewPlugin::isContainer() const {
  return false;
}

QString RotateViewPlugin::domXml() const {
  return "<widget class=\"TribotsTools::RotateView\" name=\"RotateView\">\n"
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

QString RotateViewPlugin::includeFile() const {
  return "../RotateView.h";
}

void RotateViewPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool RotateViewPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, RotateViewPlugin)
