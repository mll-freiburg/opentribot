/****************************************************************************
** TribotsTools::SLErrorWidget meta object code from reading C++ file 'SLErrorWidget.h'
**
** Created: Mon Aug 31 14:45:19 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "SLErrorWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *TribotsTools::SLErrorWidget::className() const
{
    return "TribotsTools::SLErrorWidget";
}

QMetaObject *TribotsTools::SLErrorWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsTools__SLErrorWidget( "TribotsTools::SLErrorWidget", &TribotsTools::SLErrorWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsTools::SLErrorWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::SLErrorWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsTools::SLErrorWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::SLErrorWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsTools::SLErrorWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUMethod slot_0 = {"update_error", 0, 0 };
    static const QUMethod slot_1 = {"update_optimising_pos", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "update_error()", &slot_0, QMetaData::Public },
	{ "update_optimising_pos()", &slot_1, QMetaData::Public }
    };
    static const QUMethod signal_0 = {"robot_update", 0, 0 };
    static const QMetaData signal_tbl[] = {
	{ "robot_update()", &signal_0, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsTools::SLErrorWidget", parentObject,
	slot_tbl, 2,
	signal_tbl, 1,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsTools__SLErrorWidget.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsTools::SLErrorWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsTools::SLErrorWidget" ) )
	return this;
    return QWidget::qt_cast( clname );
}

// SIGNAL robot_update
void TribotsTools::SLErrorWidget::robot_update()
{
    activate_signal( staticMetaObject()->signalOffset() + 0 );
}

bool TribotsTools::SLErrorWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: update_error(); break;
    case 1: update_optimising_pos(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsTools::SLErrorWidget::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: robot_update(); break;
    default:
	return QWidget::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool TribotsTools::SLErrorWidget::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool TribotsTools::SLErrorWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
