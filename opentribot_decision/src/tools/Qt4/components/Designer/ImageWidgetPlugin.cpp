
#include "../ImageWidget.h"
#include "ImageWidgetPlugin.h"

#include <QtCore/QtPlugin>

ImageWidgetPlugin::ImageWidgetPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *ImageWidgetPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::ImageWidget(parent);
}

QString ImageWidgetPlugin::name() const {
  return "TribotsTools::ImageWidget";
}

QString ImageWidgetPlugin::group() const {
  return "Tribots Widgets";
}

QIcon ImageWidgetPlugin::icon() const {
  return QIcon();
}

QString ImageWidgetPlugin::toolTip() const {
  return "ImageWidget";
}

QString ImageWidgetPlugin::whatsThis() const {
  return "ein Widget, um ein Tribots::Image darzustellen";
}

bool ImageWidgetPlugin::isContainer() const {
  return false;
}

QString ImageWidgetPlugin::domXml() const {
  return "<widget class=\"TribotsTools::ImageWidget\" name=\"ImageWidget\">\n"
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

QString ImageWidgetPlugin::includeFile() const {
  return "../ImageWidget.h";
}

void ImageWidgetPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool ImageWidgetPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, ImageWidgetPlugin)
