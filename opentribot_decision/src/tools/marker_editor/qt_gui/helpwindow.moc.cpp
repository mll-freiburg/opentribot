/****************************************************************************
** helpWindow meta object code from reading C++ file 'helpwindow.h'
**
** Created: Mon Aug 31 14:47:41 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "helpwindow.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *helpWindow::className() const
{
    return "helpWindow";
}

QMetaObject *helpWindow::metaObj = 0;
static QMetaObjectCleanUp cleanUp_helpWindow( "helpWindow", &helpWindow::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString helpWindow::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "helpWindow", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString helpWindow::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "helpWindow", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* helpWindow::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    metaObj = QMetaObject::new_metaobject(
	"helpWindow", parentObject,
	0, 0,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_helpWindow.setMetaObject( metaObj );
    return metaObj;
}

void* helpWindow::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "helpWindow" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool helpWindow::qt_invoke( int _id, QUObject* _o )
{
    return QWidget::qt_invoke(_id,_o);
}

bool helpWindow::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool helpWindow::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool helpWindow::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
