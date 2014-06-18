/****************************************************************************
** TribotsviewMainWidget meta object code from reading C++ file 'TribotsviewMainWidget.h'
**
** Created: Mon Aug 31 14:45:20 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "TribotsviewMainWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *TribotsviewMainWidget::className() const
{
    return "TribotsviewMainWidget";
}

QMetaObject *TribotsviewMainWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsviewMainWidget( "TribotsviewMainWidget", &TribotsviewMainWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsviewMainWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsviewMainWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsviewMainWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsviewMainWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsviewMainWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QMainWindow::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ "info_praefix", &static_QUType_ptr, "std::deque<std::string>", QUParameter::In },
	{ "config_file", &static_QUType_ptr, "std::string", QUParameter::In },
	{ "use_sync_signals1", &static_QUType_bool, 0, QUParameter::In },
	{ "use_colored_goals1", &static_QUType_bool, 0, QUParameter::In },
	{ "use_attr", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"init_field_and_streams", 5, param_slot_0 };
    static const QUMethod slot_1 = {"cycleChanged", 0, 0 };
    static const QUMethod slot_2 = {"sl_pos_changed", 0, 0 };
    static const QUMethod slot_3 = {"displayChanged", 0, 0 };
    static const QUMethod slot_4 = {"prevRefStateCycle", 0, 0 };
    static const QUMethod slot_5 = {"nextRefStateCycle", 0, 0 };
    static const QUMethod slot_6 = {"nextCycle", 0, 0 };
    static const QUMethod slot_7 = {"prevCycle", 0, 0 };
    static const QUMethod slot_8 = {"play_on", 0, 0 };
    static const QUMethod slot_9 = {"start_play", 0, 0 };
    static const QUMethod slot_10 = {"start_rew", 0, 0 };
    static const QUMethod slot_11 = {"start_ffw", 0, 0 };
    static const QUMethod slot_12 = {"start_frew", 0, 0 };
    static const QUMethod slot_13 = {"stop_play", 0, 0 };
    static const QUMethod slot_14 = {"setCycleNum", 0, 0 };
    static const QUMethod slot_15 = {"setTime", 0, 0 };
    static const QUParameter param_slot_16[] = {
	{ "b", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_16 = {"toogleImageView", 1, param_slot_16 };
    static const QUParameter param_slot_17[] = {
	{ "v", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_17 = {"change_display_frequency", 1, param_slot_17 };
    static const QUParameter param_slot_18[] = {
	{ "i", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_18 = {"cycle_slider_moved", 1, param_slot_18 };
    static const QUParameter param_slot_19[] = {
	{ "i", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_19 = {"cycle_slider_value_changed", 1, param_slot_19 };
    static const QUMethod slot_20 = {"fileExit", 0, 0 };
    static const QUMethod slot_21 = {"revert_file", 0, 0 };
    static const QUMethod slot_22 = {"reload_file", 0, 0 };
    static const QUMethod slot_23 = {"loadImages", 0, 0 };
    static const QUParameter param_slot_24[] = {
	{ "p1", &static_QUType_ptr, "Tribots::Vec", QUParameter::In },
	{ "p2", &static_QUType_ptr, "Tribots::Vec", QUParameter::In }
    };
    static const QUMethod slot_24 = {"showSLError", 2, param_slot_24 };
    static const QUMethod slot_25 = {"replaceCycleInfo", 0, 0 };
    static const QUParameter param_slot_26[] = {
	{ "event", &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_26 = {"unresolvedKeyPressEvent", 1, param_slot_26 };
    static const QUMethod slot_27 = {"show", 0, 0 };
    static const QUParameter param_slot_28[] = {
	{ "s", &static_QUType_QString, 0, QUParameter::In }
    };
    static const QUMethod slot_28 = {"displayStatusMessage", 1, param_slot_28 };
    static const QUMethod slot_29 = {"refrobotChanged", 0, 0 };
    static const QUParameter param_slot_30[] = {
	{ "b", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_30 = {"toggleColoredGaols", 1, param_slot_30 };
    static const QUMethod slot_31 = {"languageChange", 0, 0 };
    static const QUMethod slot_32 = {"goto_start", 0, 0 };
    static const QUMethod slot_33 = {"goto_end", 0, 0 };
    static const QUMethod slot_34 = {"loadAdditionalLogfile", 0, 0 };
    static const QUParameter param_slot_35[] = {
	{ "ev", &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_35 = {"keyPressEvent", 1, param_slot_35 };
    static const QUMethod slot_36 = {"buildCycleInfo", 0, 0 };
    static const QUMethod slot_37 = {"toggleBookmark", 0, 0 };
    static const QUMethod slot_38 = {"gotoNextBookmark", 0, 0 };
    static const QUMethod slot_39 = {"gotoPrevBookmark", 0, 0 };
    static const QUMethod slot_40 = {"clearBookmarks", 0, 0 };
    static const QUParameter param_slot_41[] = {
	{ "b", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_41 = {"toggleGreyRobots", 1, param_slot_41 };
    static const QUParameter param_slot_42[] = {
	{ "b", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_42 = {"toggleSyncSignals", 1, param_slot_42 };
    static const QMetaData slot_tbl[] = {
	{ "init_field_and_streams(const std::deque<std::string>&,const std::string&,bool,bool,bool)", &slot_0, QMetaData::Public },
	{ "cycleChanged()", &slot_1, QMetaData::Public },
	{ "sl_pos_changed()", &slot_2, QMetaData::Public },
	{ "displayChanged()", &slot_3, QMetaData::Public },
	{ "prevRefStateCycle()", &slot_4, QMetaData::Public },
	{ "nextRefStateCycle()", &slot_5, QMetaData::Public },
	{ "nextCycle()", &slot_6, QMetaData::Public },
	{ "prevCycle()", &slot_7, QMetaData::Public },
	{ "play_on()", &slot_8, QMetaData::Public },
	{ "start_play()", &slot_9, QMetaData::Public },
	{ "start_rew()", &slot_10, QMetaData::Public },
	{ "start_ffw()", &slot_11, QMetaData::Public },
	{ "start_frew()", &slot_12, QMetaData::Public },
	{ "stop_play()", &slot_13, QMetaData::Public },
	{ "setCycleNum()", &slot_14, QMetaData::Public },
	{ "setTime()", &slot_15, QMetaData::Public },
	{ "toogleImageView(bool)", &slot_16, QMetaData::Public },
	{ "change_display_frequency(int)", &slot_17, QMetaData::Public },
	{ "cycle_slider_moved(int)", &slot_18, QMetaData::Public },
	{ "cycle_slider_value_changed(int)", &slot_19, QMetaData::Public },
	{ "fileExit()", &slot_20, QMetaData::Public },
	{ "revert_file()", &slot_21, QMetaData::Public },
	{ "reload_file()", &slot_22, QMetaData::Public },
	{ "loadImages()", &slot_23, QMetaData::Public },
	{ "showSLError(Tribots::Vec,Tribots::Vec)", &slot_24, QMetaData::Public },
	{ "replaceCycleInfo()", &slot_25, QMetaData::Public },
	{ "unresolvedKeyPressEvent(QKeyEvent*)", &slot_26, QMetaData::Public },
	{ "show()", &slot_27, QMetaData::Public },
	{ "displayStatusMessage(QString)", &slot_28, QMetaData::Public },
	{ "refrobotChanged()", &slot_29, QMetaData::Public },
	{ "toggleColoredGaols(bool)", &slot_30, QMetaData::Public },
	{ "languageChange()", &slot_31, QMetaData::Protected },
	{ "goto_start()", &slot_32, QMetaData::Protected },
	{ "goto_end()", &slot_33, QMetaData::Protected },
	{ "loadAdditionalLogfile()", &slot_34, QMetaData::Protected },
	{ "keyPressEvent(QKeyEvent*)", &slot_35, QMetaData::Protected },
	{ "buildCycleInfo()", &slot_36, QMetaData::Protected },
	{ "toggleBookmark()", &slot_37, QMetaData::Protected },
	{ "gotoNextBookmark()", &slot_38, QMetaData::Protected },
	{ "gotoPrevBookmark()", &slot_39, QMetaData::Protected },
	{ "clearBookmarks()", &slot_40, QMetaData::Protected },
	{ "toggleGreyRobots(bool)", &slot_41, QMetaData::Private },
	{ "toggleSyncSignals(bool)", &slot_42, QMetaData::Private }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsviewMainWidget", parentObject,
	slot_tbl, 43,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsviewMainWidget.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsviewMainWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsviewMainWidget" ) )
	return this;
    return QMainWindow::qt_cast( clname );
}

bool TribotsviewMainWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: init_field_and_streams((const std::deque<std::string>&)*((const std::deque<std::string>*)static_QUType_ptr.get(_o+1)),(const std::string&)*((const std::string*)static_QUType_ptr.get(_o+2)),(bool)static_QUType_bool.get(_o+3),(bool)static_QUType_bool.get(_o+4),(bool)static_QUType_bool.get(_o+5)); break;
    case 1: cycleChanged(); break;
    case 2: sl_pos_changed(); break;
    case 3: displayChanged(); break;
    case 4: prevRefStateCycle(); break;
    case 5: nextRefStateCycle(); break;
    case 6: nextCycle(); break;
    case 7: prevCycle(); break;
    case 8: play_on(); break;
    case 9: start_play(); break;
    case 10: start_rew(); break;
    case 11: start_ffw(); break;
    case 12: start_frew(); break;
    case 13: stop_play(); break;
    case 14: setCycleNum(); break;
    case 15: setTime(); break;
    case 16: toogleImageView((bool)static_QUType_bool.get(_o+1)); break;
    case 17: change_display_frequency((int)static_QUType_int.get(_o+1)); break;
    case 18: cycle_slider_moved((int)static_QUType_int.get(_o+1)); break;
    case 19: cycle_slider_value_changed((int)static_QUType_int.get(_o+1)); break;
    case 20: fileExit(); break;
    case 21: revert_file(); break;
    case 22: reload_file(); break;
    case 23: loadImages(); break;
    case 24: showSLError((Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+1))),(Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+2)))); break;
    case 25: replaceCycleInfo(); break;
    case 26: unresolvedKeyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 27: show(); break;
    case 28: displayStatusMessage((QString)static_QUType_QString.get(_o+1)); break;
    case 29: refrobotChanged(); break;
    case 30: toggleColoredGaols((bool)static_QUType_bool.get(_o+1)); break;
    case 31: languageChange(); break;
    case 32: goto_start(); break;
    case 33: goto_end(); break;
    case 34: loadAdditionalLogfile(); break;
    case 35: keyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 36: buildCycleInfo(); break;
    case 37: toggleBookmark(); break;
    case 38: gotoNextBookmark(); break;
    case 39: gotoPrevBookmark(); break;
    case 40: clearBookmarks(); break;
    case 41: toggleGreyRobots((bool)static_QUType_bool.get(_o+1)); break;
    case 42: toggleSyncSignals((bool)static_QUType_bool.get(_o+1)); break;
    default:
	return QMainWindow::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsviewMainWidget::qt_emit( int _id, QUObject* _o )
{
    return QMainWindow::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool TribotsviewMainWidget::qt_property( int id, int f, QVariant* v)
{
    return QMainWindow::qt_property( id, f, v);
}

bool TribotsviewMainWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
