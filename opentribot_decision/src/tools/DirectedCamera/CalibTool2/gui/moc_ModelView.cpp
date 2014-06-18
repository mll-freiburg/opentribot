/****************************************************************************
** ModelView meta object code from reading C++ file 'ModelView.h'
**
** Created: Thu Aug 17 12:33:36 2006
**      by: The Qt MOC ($Id: moc_ModelView.cpp,v 1.1.1.1 2007-05-04 14:20:26 tribots Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "ModelView.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *ModelView::className() const
{
    return "ModelView";
}

QMetaObject *ModelView::metaObj = 0;
static QMetaObjectCleanUp cleanUp_ModelView( "ModelView", &ModelView::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString ModelView::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ModelView", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString ModelView::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ModelView", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* ModelView::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QDialog::staticMetaObject();
    static const QUMethod slot_0 = {"showNextPoint", 0, 0 };
    static const QUMethod slot_1 = {"showPreviousPoint", 0, 0 };
    static const QUMethod slot_2 = {"deleteActPoint", 0, 0 };
    static const QUMethod slot_3 = {"languageChange", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "showNextPoint()", &slot_0, QMetaData::Public },
	{ "showPreviousPoint()", &slot_1, QMetaData::Public },
	{ "deleteActPoint()", &slot_2, QMetaData::Public },
	{ "languageChange()", &slot_3, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"ModelView", parentObject,
	slot_tbl, 4,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_ModelView.setMetaObject( metaObj );
    return metaObj;
}

void* ModelView::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "ModelView" ) )
	return this;
    return QDialog::qt_cast( clname );
}

bool ModelView::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: showNextPoint(); break;
    case 1: showPreviousPoint(); break;
    case 2: deleteActPoint(); break;
    case 3: languageChange(); break;
    default:
	return QDialog::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool ModelView::qt_emit( int _id, QUObject* _o )
{
    return QDialog::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool ModelView::qt_property( int id, int f, QVariant* v)
{
    return QDialog::qt_property( id, f, v);
}

bool ModelView::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
