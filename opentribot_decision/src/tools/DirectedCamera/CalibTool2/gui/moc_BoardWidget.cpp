/****************************************************************************
** BoardWidget meta object code from reading C++ file 'BoardWidget.h'
**
** Created: Thu Aug 17 12:33:32 2006
**      by: The Qt MOC ($Id: moc_BoardWidget.cpp,v 1.1.1.1 2007-05-04 14:20:26 tribots Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "BoardWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *BoardWidget::className() const
{
    return "BoardWidget";
}

QMetaObject *BoardWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_BoardWidget( "BoardWidget", &BoardWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString BoardWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "BoardWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString BoardWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "BoardWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* BoardWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QFrame::staticMetaObject();
    metaObj = QMetaObject::new_metaobject(
	"BoardWidget", parentObject,
	0, 0,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_BoardWidget.setMetaObject( metaObj );
    return metaObj;
}

void* BoardWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "BoardWidget" ) )
	return this;
    return QFrame::qt_cast( clname );
}

bool BoardWidget::qt_invoke( int _id, QUObject* _o )
{
    return QFrame::qt_invoke(_id,_o);
}

bool BoardWidget::qt_emit( int _id, QUObject* _o )
{
    return QFrame::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool BoardWidget::qt_property( int id, int f, QVariant* v)
{
    return QFrame::qt_property( id, f, v);
}

bool BoardWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
