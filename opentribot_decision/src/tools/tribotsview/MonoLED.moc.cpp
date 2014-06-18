/****************************************************************************
** TribotsTools::MonoLED meta object code from reading C++ file 'MonoLED.h'
**
** Created: Mon Aug 31 14:47:13 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "MonoLED.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *TribotsTools::MonoLED::className() const
{
    return "TribotsTools::MonoLED";
}

QMetaObject *TribotsTools::MonoLED::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsTools__MonoLED( "TribotsTools::MonoLED", &TribotsTools::MonoLED::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsTools::MonoLED::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::MonoLED", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsTools::MonoLED::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::MonoLED", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsTools::MonoLED::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"setOn", 1, param_slot_0 };
    static const QUParameter param_slot_1[] = {
	{ 0, &static_QUType_ptr, "QPaintEvent", QUParameter::In }
    };
    static const QUMethod slot_1 = {"paintEvent", 1, param_slot_1 };
    static const QMetaData slot_tbl[] = {
	{ "setOn(bool)", &slot_0, QMetaData::Public },
	{ "paintEvent(QPaintEvent*)", &slot_1, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsTools::MonoLED", parentObject,
	slot_tbl, 2,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsTools__MonoLED.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsTools::MonoLED::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsTools::MonoLED" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool TribotsTools::MonoLED::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: setOn((bool)static_QUType_bool.get(_o+1)); break;
    case 1: paintEvent((QPaintEvent*)static_QUType_ptr.get(_o+1)); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsTools::MonoLED::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool TribotsTools::MonoLED::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool TribotsTools::MonoLED::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
