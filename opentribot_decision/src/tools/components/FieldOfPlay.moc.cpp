/****************************************************************************
** TribotsTools::FieldOfPlay meta object code from reading C++ file 'FieldOfPlay.h'
**
** Created: Mon Aug 31 14:45:20 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "FieldOfPlay.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *TribotsTools::FieldOfPlay::className() const
{
    return "TribotsTools::FieldOfPlay";
}

QMetaObject *TribotsTools::FieldOfPlay::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsTools__FieldOfPlay( "TribotsTools::FieldOfPlay", &TribotsTools::FieldOfPlay::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsTools::FieldOfPlay::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::FieldOfPlay", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsTools::FieldOfPlay::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::FieldOfPlay", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsTools::FieldOfPlay::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ 0, &static_QUType_ptr, "CycleInfo", QUParameter::In }
    };
    static const QUMethod slot_0 = {"next_cycle", 1, param_slot_0 };
    static const QUParameter param_slot_1[] = {
	{ 0, &static_QUType_ptr, "Tribots::Vec", QUParameter::In }
    };
    static const QUMethod slot_1 = {"move_clipping", 1, param_slot_1 };
    static const QUParameter param_slot_2[] = {
	{ 0, &static_QUType_ptr, "Tribots::Vec", QUParameter::In },
	{ 0, &static_QUType_ptr, "Tribots::Vec", QUParameter::In }
    };
    static const QUMethod slot_2 = {"zoom_rect", 2, param_slot_2 };
    static const QUMethod slot_3 = {"zoom_in", 0, 0 };
    static const QUMethod slot_4 = {"zoom_out", 0, 0 };
    static const QUMethod slot_5 = {"zoom_all", 0, 0 };
    static const QUMethod slot_6 = {"zoom_undo", 0, 0 };
    static const QUMethod slot_7 = {"zoom_redo", 0, 0 };
    static const QUMethod slot_8 = {"flip_side", 0, 0 };
    static const QUMethod slot_9 = {"flip_goals", 0, 0 };
    static const QUParameter param_slot_10[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_10 = {"show_wm_robot", 1, param_slot_10 };
    static const QUParameter param_slot_11[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_11 = {"show_wm_ball", 1, param_slot_11 };
    static const QUParameter param_slot_12[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_12 = {"show_wm_obstacles", 1, param_slot_12 };
    static const QUParameter param_slot_13[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_13 = {"show_vis_lines", 1, param_slot_13 };
    static const QUParameter param_slot_14[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_14 = {"show_vis_ball", 1, param_slot_14 };
    static const QUParameter param_slot_15[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_15 = {"show_vis_obstacles", 1, param_slot_15 };
    static const QUParameter param_slot_16[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_16 = {"show_vis_goals", 1, param_slot_16 };
    static const QUParameter param_slot_17[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_17 = {"show_sl_pos", 1, param_slot_17 };
    static const QUParameter param_slot_18[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_18 = {"show_robot_trace", 1, param_slot_18 };
    static const QUParameter param_slot_19[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_19 = {"show_colored_goals", 1, param_slot_19 };
    static const QUParameter param_slot_20[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_20 = {"show_direction", 1, param_slot_20 };
    static const QUMethod slot_21 = {"toggle_robot_trace", 0, 0 };
    static const QUParameter param_slot_22[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_22 = {"show_ball_trace", 1, param_slot_22 };
    static const QUParameter param_slot_23[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_23 = {"show_robot_ids", 1, param_slot_23 };
    static const QUParameter param_slot_24[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_24 = {"show_robot_ball_links", 1, param_slot_24 };
    static const QUParameter param_slot_25[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_25 = {"show_aux_lines", 1, param_slot_25 };
    static const QUMethod slot_26 = {"clear_lines", 0, 0 };
    static const QUParameter param_slot_27[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_27 = {"use_exec_time", 1, param_slot_27 };
    static const QUMethod slot_28 = {"next_refrobot", 0, 0 };
    static const QUMethod slot_29 = {"next_imagesource", 0, 0 };
    static const QUMethod slot_30 = {"say", 0, 0 };
    static const QUParameter param_slot_31[] = {
	{ 0, &static_QUType_ptr, "QPaintEvent", QUParameter::In }
    };
    static const QUMethod slot_31 = {"paintEvent", 1, param_slot_31 };
    static const QUParameter param_slot_32[] = {
	{ 0, &static_QUType_ptr, "QResizeEvent", QUParameter::In }
    };
    static const QUMethod slot_32 = {"resizeEvent", 1, param_slot_32 };
    static const QUParameter param_slot_33[] = {
	{ 0, &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_33 = {"keyPressEvent", 1, param_slot_33 };
    static const QUParameter param_slot_34[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_34 = {"mouseMoveEvent", 1, param_slot_34 };
    static const QUParameter param_slot_35[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_35 = {"mousePressEvent", 1, param_slot_35 };
    static const QUParameter param_slot_36[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_36 = {"mouseReleaseEvent", 1, param_slot_36 };
    static const QUMethod slot_37 = {"clippingChanged", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "next_cycle(const CycleInfo&)", &slot_0, QMetaData::Public },
	{ "move_clipping(Tribots::Vec)", &slot_1, QMetaData::Public },
	{ "zoom_rect(Tribots::Vec,Tribots::Vec)", &slot_2, QMetaData::Public },
	{ "zoom_in()", &slot_3, QMetaData::Public },
	{ "zoom_out()", &slot_4, QMetaData::Public },
	{ "zoom_all()", &slot_5, QMetaData::Public },
	{ "zoom_undo()", &slot_6, QMetaData::Public },
	{ "zoom_redo()", &slot_7, QMetaData::Public },
	{ "flip_side()", &slot_8, QMetaData::Public },
	{ "flip_goals()", &slot_9, QMetaData::Public },
	{ "show_wm_robot(bool)", &slot_10, QMetaData::Public },
	{ "show_wm_ball(bool)", &slot_11, QMetaData::Public },
	{ "show_wm_obstacles(bool)", &slot_12, QMetaData::Public },
	{ "show_vis_lines(bool)", &slot_13, QMetaData::Public },
	{ "show_vis_ball(bool)", &slot_14, QMetaData::Public },
	{ "show_vis_obstacles(bool)", &slot_15, QMetaData::Public },
	{ "show_vis_goals(bool)", &slot_16, QMetaData::Public },
	{ "show_sl_pos(bool)", &slot_17, QMetaData::Public },
	{ "show_robot_trace(bool)", &slot_18, QMetaData::Public },
	{ "show_colored_goals(bool)", &slot_19, QMetaData::Public },
	{ "show_direction(bool)", &slot_20, QMetaData::Public },
	{ "toggle_robot_trace()", &slot_21, QMetaData::Public },
	{ "show_ball_trace(bool)", &slot_22, QMetaData::Public },
	{ "show_robot_ids(bool)", &slot_23, QMetaData::Public },
	{ "show_robot_ball_links(bool)", &slot_24, QMetaData::Public },
	{ "show_aux_lines(bool)", &slot_25, QMetaData::Public },
	{ "clear_lines()", &slot_26, QMetaData::Public },
	{ "use_exec_time(bool)", &slot_27, QMetaData::Public },
	{ "next_refrobot()", &slot_28, QMetaData::Public },
	{ "next_imagesource()", &slot_29, QMetaData::Public },
	{ "say()", &slot_30, QMetaData::Public },
	{ "paintEvent(QPaintEvent*)", &slot_31, QMetaData::Protected },
	{ "resizeEvent(QResizeEvent*)", &slot_32, QMetaData::Protected },
	{ "keyPressEvent(QKeyEvent*)", &slot_33, QMetaData::Protected },
	{ "mouseMoveEvent(QMouseEvent*)", &slot_34, QMetaData::Protected },
	{ "mousePressEvent(QMouseEvent*)", &slot_35, QMetaData::Protected },
	{ "mouseReleaseEvent(QMouseEvent*)", &slot_36, QMetaData::Protected },
	{ "clippingChanged()", &slot_37, QMetaData::Protected }
    };
    static const QUMethod signal_0 = {"slDisplacement", 0, 0 };
    static const QUParameter param_signal_1[] = {
	{ 0, &static_QUType_ptr, "Tribots::Vec", QUParameter::In },
	{ 0, &static_QUType_ptr, "Tribots::Angle", QUParameter::In }
    };
    static const QUMethod signal_1 = {"robotDisplacement", 2, param_signal_1 };
    static const QUMethod signal_2 = {"preferencesChanged", 0, 0 };
    static const QUMethod signal_3 = {"refrobotChanged", 0, 0 };
    static const QUMethod signal_4 = {"cycleChanged", 0, 0 };
    static const QUParameter param_signal_5[] = {
	{ 0, &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod signal_5 = {"unresolvedKeyPressEvent", 1, param_signal_5 };
    static const QUParameter param_signal_6[] = {
	{ 0, &static_QUType_ptr, "Tribots::Vec", QUParameter::In },
	{ 0, &static_QUType_ptr, "Tribots::Vec", QUParameter::In }
    };
    static const QUMethod signal_6 = {"unresolvedMouseRect", 2, param_signal_6 };
    static const QUParameter param_signal_7[] = {
	{ 0, &static_QUType_QString, 0, QUParameter::In }
    };
    static const QUMethod signal_7 = {"vectorMessage", 1, param_signal_7 };
    static const QMetaData signal_tbl[] = {
	{ "slDisplacement()", &signal_0, QMetaData::Public },
	{ "robotDisplacement(Tribots::Vec,Tribots::Angle)", &signal_1, QMetaData::Public },
	{ "preferencesChanged()", &signal_2, QMetaData::Public },
	{ "refrobotChanged()", &signal_3, QMetaData::Public },
	{ "cycleChanged()", &signal_4, QMetaData::Public },
	{ "unresolvedKeyPressEvent(QKeyEvent*)", &signal_5, QMetaData::Public },
	{ "unresolvedMouseRect(Tribots::Vec,Tribots::Vec)", &signal_6, QMetaData::Public },
	{ "vectorMessage(QString)", &signal_7, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsTools::FieldOfPlay", parentObject,
	slot_tbl, 38,
	signal_tbl, 8,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsTools__FieldOfPlay.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsTools::FieldOfPlay::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsTools::FieldOfPlay" ) )
	return this;
    return QWidget::qt_cast( clname );
}

// SIGNAL slDisplacement
void TribotsTools::FieldOfPlay::slDisplacement()
{
    activate_signal( staticMetaObject()->signalOffset() + 0 );
}

#include <qobjectdefs.h>
#include <qsignalslotimp.h>

// SIGNAL robotDisplacement
void TribotsTools::FieldOfPlay::robotDisplacement( Tribots::Vec t0, Tribots::Angle t1 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 1 );
    if ( !clist )
	return;
    QUObject o[3];
    static_QUType_ptr.set(o+1,&t0);
    static_QUType_ptr.set(o+2,&t1);
    activate_signal( clist, o );
}

// SIGNAL preferencesChanged
void TribotsTools::FieldOfPlay::preferencesChanged()
{
    activate_signal( staticMetaObject()->signalOffset() + 2 );
}

// SIGNAL refrobotChanged
void TribotsTools::FieldOfPlay::refrobotChanged()
{
    activate_signal( staticMetaObject()->signalOffset() + 3 );
}

// SIGNAL cycleChanged
void TribotsTools::FieldOfPlay::cycleChanged()
{
    activate_signal( staticMetaObject()->signalOffset() + 4 );
}

// SIGNAL unresolvedKeyPressEvent
void TribotsTools::FieldOfPlay::unresolvedKeyPressEvent( QKeyEvent* t0 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 5 );
    if ( !clist )
	return;
    QUObject o[2];
    static_QUType_ptr.set(o+1,t0);
    activate_signal( clist, o );
}

// SIGNAL unresolvedMouseRect
void TribotsTools::FieldOfPlay::unresolvedMouseRect( Tribots::Vec t0, Tribots::Vec t1 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 6 );
    if ( !clist )
	return;
    QUObject o[3];
    static_QUType_ptr.set(o+1,&t0);
    static_QUType_ptr.set(o+2,&t1);
    activate_signal( clist, o );
}

// SIGNAL vectorMessage
void TribotsTools::FieldOfPlay::vectorMessage( QString t0 )
{
    activate_signal( staticMetaObject()->signalOffset() + 7, t0 );
}

bool TribotsTools::FieldOfPlay::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: next_cycle((const CycleInfo&)*((const CycleInfo*)static_QUType_ptr.get(_o+1))); break;
    case 1: move_clipping((Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+1)))); break;
    case 2: zoom_rect((Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+1))),(Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+2)))); break;
    case 3: zoom_in(); break;
    case 4: zoom_out(); break;
    case 5: zoom_all(); break;
    case 6: zoom_undo(); break;
    case 7: zoom_redo(); break;
    case 8: flip_side(); break;
    case 9: flip_goals(); break;
    case 10: show_wm_robot((bool)static_QUType_bool.get(_o+1)); break;
    case 11: show_wm_ball((bool)static_QUType_bool.get(_o+1)); break;
    case 12: show_wm_obstacles((bool)static_QUType_bool.get(_o+1)); break;
    case 13: show_vis_lines((bool)static_QUType_bool.get(_o+1)); break;
    case 14: show_vis_ball((bool)static_QUType_bool.get(_o+1)); break;
    case 15: show_vis_obstacles((bool)static_QUType_bool.get(_o+1)); break;
    case 16: show_vis_goals((bool)static_QUType_bool.get(_o+1)); break;
    case 17: show_sl_pos((bool)static_QUType_bool.get(_o+1)); break;
    case 18: show_robot_trace((bool)static_QUType_bool.get(_o+1)); break;
    case 19: show_colored_goals((bool)static_QUType_bool.get(_o+1)); break;
    case 20: show_direction((bool)static_QUType_bool.get(_o+1)); break;
    case 21: toggle_robot_trace(); break;
    case 22: show_ball_trace((bool)static_QUType_bool.get(_o+1)); break;
    case 23: show_robot_ids((bool)static_QUType_bool.get(_o+1)); break;
    case 24: show_robot_ball_links((bool)static_QUType_bool.get(_o+1)); break;
    case 25: show_aux_lines((bool)static_QUType_bool.get(_o+1)); break;
    case 26: clear_lines(); break;
    case 27: use_exec_time((bool)static_QUType_bool.get(_o+1)); break;
    case 28: next_refrobot(); break;
    case 29: next_imagesource(); break;
    case 30: say(); break;
    case 31: paintEvent((QPaintEvent*)static_QUType_ptr.get(_o+1)); break;
    case 32: resizeEvent((QResizeEvent*)static_QUType_ptr.get(_o+1)); break;
    case 33: keyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 34: mouseMoveEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 35: mousePressEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 36: mouseReleaseEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 37: clippingChanged(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsTools::FieldOfPlay::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: slDisplacement(); break;
    case 1: robotDisplacement((Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+1))),(Tribots::Angle)(*((Tribots::Angle*)static_QUType_ptr.get(_o+2)))); break;
    case 2: preferencesChanged(); break;
    case 3: refrobotChanged(); break;
    case 4: cycleChanged(); break;
    case 5: unresolvedKeyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 6: unresolvedMouseRect((Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+1))),(Tribots::Vec)(*((Tribots::Vec*)static_QUType_ptr.get(_o+2)))); break;
    case 7: vectorMessage((QString)static_QUType_QString.get(_o+1)); break;
    default:
	return QWidget::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool TribotsTools::FieldOfPlay::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool TribotsTools::FieldOfPlay::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
