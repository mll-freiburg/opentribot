/****************************************************************************
** MainWidget meta object code from reading C++ file 'MainWidget.h'
**
** Created: Thu Aug 17 12:33:34 2006
**      by: The Qt MOC ($Id: moc_MainWidget.cpp,v 1.1.1.1 2007-05-04 14:20:26 tribots Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "MainWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *MainWidget::className() const
{
    return "MainWidget";
}

QMetaObject *MainWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_MainWidget( "MainWidget", &MainWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString MainWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MainWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString MainWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MainWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* MainWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUMethod slot_0 = {"tabchanged", 0, 0 };
    static const QUMethod slot_1 = {"languageChange", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "tabchanged()", &slot_0, QMetaData::Public },
	{ "languageChange()", &slot_1, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"MainWidget", parentObject,
	slot_tbl, 2,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_MainWidget.setMetaObject( metaObj );
    return metaObj;
}

void* MainWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "MainWidget" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool MainWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: tabchanged(); break;
    case 1: languageChange(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool MainWidget::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool MainWidget::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool MainWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
