/****************************************************************************
** ImageviewWidget meta object code from reading C++ file 'ImageviewWidget.h'
**
** Created: Mon Aug 31 14:45:21 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "ImageviewWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *ImageviewWidget::className() const
{
    return "ImageviewWidget";
}

QMetaObject *ImageviewWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_ImageviewWidget( "ImageviewWidget", &ImageviewWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString ImageviewWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ImageviewWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString ImageviewWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ImageviewWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* ImageviewWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ "cycle", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"showImage", 1, param_slot_0 };
    static const QUParameter param_slot_1[] = {
	{ "b", &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_1 = {"toggleFreeze", 1, param_slot_1 };
    static const QUParameter param_slot_2[] = {
	{ "filename", &static_QUType_QString, 0, QUParameter::In },
	{ "cycle", &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_2 = {"loadImages", 2, param_slot_2 };
    static const QUMethod slot_3 = {"languageChange", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "showImage(int)", &slot_0, QMetaData::Public },
	{ "toggleFreeze(bool)", &slot_1, QMetaData::Public },
	{ "loadImages(QString,int)", &slot_2, QMetaData::Public },
	{ "languageChange()", &slot_3, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"ImageviewWidget", parentObject,
	slot_tbl, 4,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_ImageviewWidget.setMetaObject( metaObj );
    return metaObj;
}

void* ImageviewWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "ImageviewWidget" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool ImageviewWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: showImage((int)static_QUType_int.get(_o+1)); break;
    case 1: toggleFreeze((bool)static_QUType_bool.get(_o+1)); break;
    case 2: loadImages((QString)static_QUType_QString.get(_o+1),(int)static_QUType_int.get(_o+2)); break;
    case 3: languageChange(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool ImageviewWidget::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool ImageviewWidget::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool ImageviewWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
