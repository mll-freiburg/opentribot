/****************************************************************************
** Paper meta object code from reading C++ file 'paper.h'
**
** Created: Mon Aug 31 14:47:41 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "paper.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *Paper::className() const
{
    return "Paper";
}

QMetaObject *Paper::metaObj = 0;
static QMetaObjectCleanUp cleanUp_Paper( "Paper", &Paper::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString Paper::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "Paper", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString Paper::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "Paper", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* Paper::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUMethod slot_0 = {"toogleBW", 0, 0 };
    static const QUMethod slot_1 = {"toogleWB", 0, 0 };
    static const QUMethod slot_2 = {"toogleWR", 0, 0 };
    static const QUMethod slot_3 = {"toogleRM", 0, 0 };
    static const QUMethod slot_4 = {"toogleRW", 0, 0 };
    static const QUMethod slot_5 = {"deleteMarkedMarkers", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "toogleBW()", &slot_0, QMetaData::Public },
	{ "toogleWB()", &slot_1, QMetaData::Public },
	{ "toogleWR()", &slot_2, QMetaData::Public },
	{ "toogleRM()", &slot_3, QMetaData::Public },
	{ "toogleRW()", &slot_4, QMetaData::Public },
	{ "deleteMarkedMarkers()", &slot_5, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"Paper", parentObject,
	slot_tbl, 6,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_Paper.setMetaObject( metaObj );
    return metaObj;
}

void* Paper::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "Paper" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool Paper::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: toogleBW(); break;
    case 1: toogleWB(); break;
    case 2: toogleWR(); break;
    case 3: toogleRM(); break;
    case 4: toogleRW(); break;
    case 5: deleteMarkedMarkers(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool Paper::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool Paper::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool Paper::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
