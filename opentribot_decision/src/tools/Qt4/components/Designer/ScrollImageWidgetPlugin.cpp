
#include "../ScrollImageWidget.h"
#include "ScrollImageWidgetPlugin.h"

#include <QtCore/QtPlugin>

ScrollImageWidgetPlugin::ScrollImageWidgetPlugin(QObject *parent) : QObject(parent), initialized(false) {;}

QWidget *ScrollImageWidgetPlugin::createWidget(QWidget *parent) {
  return new TribotsTools::ScrollImageWidget(parent);
}

QString ScrollImageWidgetPlugin::name() const {
  return "TribotsTools::ScrollImageWidget";
}

QString ScrollImageWidgetPlugin::group() const {
  return "Tribots Widgets";
}

QIcon ScrollImageWidgetPlugin::icon() const {
  return QIcon();
}

QString ScrollImageWidgetPlugin::toolTip() const {
  return "ScrollImageWidget";
}

QString ScrollImageWidgetPlugin::whatsThis() const {
  return "ein Widget mit Scrollbalken, um ein Tribots::Image darzustellen";
}

bool ScrollImageWidgetPlugin::isContainer() const {
  return false;
}

QString ScrollImageWidgetPlugin::domXml() const {
  return "<widget class=\"TribotsTools::ScrollImageWidget\" name=\"ScrollImageWidget\">\n"
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

QString ScrollImageWidgetPlugin::includeFile() const {
  return "../ScrollImageWidget.h";
}

void ScrollImageWidgetPlugin::initialize(QDesignerFormEditorInterface * /* core */) {
  if (initialized)
    return;
  initialized = true;
}

bool ScrollImageWidgetPlugin::isInitialized() const {
  return initialized;
}

Q_EXPORT_PLUGIN2(customwidgetplugin, ScrollImageWidgetPlugin)
