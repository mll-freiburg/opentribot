/****************************************************************************
** ControlGUI meta object code from reading C++ file 'ControlGUI.h'
**
** Created: Mon Aug 31 14:47:14 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "ControlGUI.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *ControlGUI::className() const
{
    return "ControlGUI";
}

QMetaObject *ControlGUI::metaObj = 0;
static QMetaObjectCleanUp cleanUp_ControlGUI( "ControlGUI", &ControlGUI::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString ControlGUI::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ControlGUI", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString ControlGUI::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ControlGUI", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* ControlGUI::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QMainWindow::staticMetaObject();
    static const QUMethod slot_0 = {"init", 0, 0 };
    static const QUMethod slot_1 = {"destroy", 0, 0 };
    static const QUParameter param_slot_2[] = {
	{ "s", &static_QUType_QString, 0, QUParameter::In }
    };
    static const QUMethod slot_2 = {"playerType", 1, param_slot_2 };
    static const QUParameter param_slot_3[] = {
	{ "s", &static_QUType_QString, 0, QUParameter::In }
    };
    static const QUMethod slot_3 = {"playerRole", 1, param_slot_3 };
    static const QUParameter param_slot_4[] = {
	{ "s", &static_QUType_QString, 0, QUParameter::In }
    };
    static const QUMethod slot_4 = {"refereeState", 1, param_slot_4 };
    static const QUMethod slot_5 = {"slHintClicked", 0, 0 };
    static const QUMethod slot_6 = {"slHintActivated", 0, 0 };
    static const QUParameter param_slot_7[] = {
	{ "ev", &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_7 = {"keyPressEvent", 1, param_slot_7 };
    static const QUParameter param_slot_8[] = {
	{ 0, &static_QUType_ptr, "Tribots::Vec", QUParameter::In },
	{ 0, &static_QUType_ptr, "Tribots::Angle", QUParameter::In }
    };
    static const QUMethod slot_8 = {"robotGoTo", 2, param_slot_8 };
    static const QUMethod slot_9 = {"gotoActivated", 0, 0 };
    static const QUMethod slot_10 = {"languageChange", 0, 0 };
    static const QUMethod slot_11 = {"activate", 0, 0 };
    static const QUMethod slot_12 = {"deactivate", 0, 0 };
    static const QUMethod slot_13 = {"debugImage", 0, 0 };
    static const QUMethod slot_14 = {"cycleTask", 0, 0 };
    static const QUMethod slot_15 = {"send", 0, 0 };
    static const QUMethod slot_16 = {"receive", 0, 0 };
    static const QUMethod slot_17 = {"updateDisplay", 0, 0 };
    static const QUMethod slot_18 = {"execRobotcontrol", 0, 0 };
    static const QUMethod slot_19 = {"quitRobotcontrol", 0, 0 };
    static const QUMethod slot_20 = {"execRestartCaller", 0, 0 };
    static const QUMethod slot_21 = {"execColorToolOmni", 0, 0 };
    static const QUMethod slot_22 = {"execColorToolPerspective", 0, 0 };
    static const QUMethod slot_23 = {"execCalibrationTool", 0, 0 };
    static const QUMethod slot_24 = {"execMarkerEditor", 0, 0 };
    static const QUMethod slot_25 = {"execCoriander", 0, 0 };
    static const QUMethod slot_26 = {"execConfigEditor", 0, 0 };
    static const QUMethod slot_27 = {"execJournalViewer", 0, 0 };
    static const QUMethod slot_28 = {"execTribotsview", 0, 0 };
    static const QUParameter param_slot_29[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_29 = {"toggleBlackScreen", 1, param_slot_29 };
    static const QUMethod slot_30 = {"killChild", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "init()", &slot_0, QMetaData::Public },
	{ "destroy()", &slot_1, QMetaData::Public },
	{ "playerType(const QString&)", &slot_2, QMetaData::Public },
	{ "playerRole(const QString&)", &slot_3, QMetaData::Public },
	{ "refereeState(const QString&)", &slot_4, QMetaData::Public },
	{ "slHintClicked()", &slot_5, QMetaData::Public },
	{ "slHintActivated()", &slot_6, QMetaData::Public },
	{ "keyPressEvent(QKeyEvent*)", &slot_7, QMetaData::Public },
	{ "robotGoTo(Tribots::Vec,Tribots::Angle)", &slot_8, QMetaData::Public },
	{ "gotoActivated()", &slot_9, QMetaData::Public },
	{ "languageChange()", &slot_10, QMetaData::Protected },
	{ "activate()", &slot_11, QMetaData::Private },
	{ "deactivate()", &slot_12, QMetaData::Private },
	{ "debugImage()", &slot_13, QMetaData::Private },
	{ "cycleTask()", &slot_14, QMetaData::Private },
	{ "send()", &slot_15, QMetaData::Private },
	{ "receive()", &slot_16, QMetaData::Private },
	{ "updateDisplay()", &slot_17, QMetaData::Private },
	{ "execRobotcontrol()", &slot_18, QMetaData::Private },
	{ "quitRobotcontrol()", &slot_19, QMetaData::Private },
	{ "execRestartCaller()", &slot_20, QMetaData::Private },
	{ "execColorToolOmni()", &slot_21, QMetaData::Private },
	{ "execColorToolPerspective()", &slot_22, QMetaData::Private },
	{ "execCalibrationTool()", &slot_23, QMetaData::Private },
	{ "execMarkerEditor()", &slot_24, QMetaData::Private },
	{ "execCoriander()", &slot_25, QMetaData::Private },
	{ "execConfigEditor()", &slot_26, QMetaData::Private },
	{ "execJournalViewer()", &slot_27, QMetaData::Private },
	{ "execTribotsview()", &slot_28, QMetaData::Private },
	{ "toggleBlackScreen(bool)", &slot_29, QMetaData::Private },
	{ "killChild()", &slot_30, QMetaData::Private }
    };
    metaObj = QMetaObject::new_metaobject(
	"ControlGUI", parentObject,
	slot_tbl, 31,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_ControlGUI.setMetaObject( metaObj );
    return metaObj;
}

void* ControlGUI::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "ControlGUI" ) )
	return this;
    return QMainWindow::qt_cast( clname );
}

bool ControlGUI::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: init(); break;
    case 1: destroy(); break;
    case 2: playerType((const QString&)static_QUType_QString.get(_o+1)); break;
    case 3: playerRole((const QString&)static_QUType_QString.get(_o+1)); break;
    case 4: refereeState((const QString&)static_QUType_QString.get(_o+1)); break;
    case 5: slHintClicked(); break;
    case 6: slHintActivated(); break;
    case 7: keyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 8: robotGoTo((Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+1))),(Tribots::Angle)(*((Tribots::Angle*)static_QUType_ptr.get(_o+2)))); break;
    case 9: gotoActivated(); break;
    case 10: languageChange(); break;
    case 11: activate(); break;
    case 12: deactivate(); break;
    case 13: debugImage(); break;
    case 14: cycleTask(); break;
    case 15: send(); break;
    case 16: receive(); break;
    case 17: updateDisplay(); break;
    case 18: execRobotcontrol(); break;
    case 19: quitRobotcontrol(); break;
    case 20: execRestartCaller(); break;
    case 21: execColorToolOmni(); break;
    case 22: execColorToolPerspective(); break;
    case 23: execCalibrationTool(); break;
    case 24: execMarkerEditor(); break;
    case 25: execCoriander(); break;
    case 26: execConfigEditor(); break;
    case 27: execJournalViewer(); break;
    case 28: execTribotsview(); break;
    case 29: toggleBlackScreen((bool)static_QUType_bool.get(_o+1)); break;
    case 30: killChild(); break;
    default:
	return QMainWindow::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool ControlGUI::qt_emit( int _id, QUObject* _o )
{
    return QMainWindow::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool ControlGUI::qt_property( int id, int f, QVariant* v)
{
    return QMainWindow::qt_property( id, f, v);
}

bool ControlGUI::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
