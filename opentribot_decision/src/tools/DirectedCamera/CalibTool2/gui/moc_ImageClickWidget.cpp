/****************************************************************************
** ImageClickWidget meta object code from reading C++ file 'ImageClickWidget.h'
**
** Created: Thu Aug 17 12:33:33 2006
**      by: The Qt MOC ($Id: moc_ImageClickWidget.cpp,v 1.1.1.1 2007-05-04 14:20:26 tribots Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "ImageClickWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *ImageClickWidget::className() const
{
    return "ImageClickWidget";
}

QMetaObject *ImageClickWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_ImageClickWidget( "ImageClickWidget", &ImageClickWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString ImageClickWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ImageClickWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString ImageClickWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ImageClickWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* ImageClickWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = TribotsTools::ImageWidget::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ "e", &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_0 = {"mouseMoveEvent", 1, param_slot_0 };
    static const QUParameter param_slot_1[] = {
	{ "e", &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_1 = {"mousePressEvent", 1, param_slot_1 };
    static const QUParameter param_slot_2[] = {
	{ "e", &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_2 = {"keyPressEvent", 1, param_slot_2 };
    static const QUParameter param_slot_3[] = {
	{ "e", &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_3 = {"keyReleaseEvent", 1, param_slot_3 };
    static const QMetaData slot_tbl[] = {
	{ "mouseMoveEvent(QMouseEvent*)", &slot_0, QMetaData::Public },
	{ "mousePressEvent(QMouseEvent*)", &slot_1, QMetaData::Public },
	{ "keyPressEvent(QKeyEvent*)", &slot_2, QMetaData::Public },
	{ "keyReleaseEvent(QKeyEvent*)", &slot_3, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"ImageClickWidget", parentObject,
	slot_tbl, 4,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_ImageClickWidget.setMetaObject( metaObj );
    return metaObj;
}

void* ImageClickWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "ImageClickWidget" ) )
	return this;
    return ImageWidget::qt_cast( clname );
}

bool ImageClickWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: mouseMoveEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 1: mousePressEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 2: keyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 3: keyReleaseEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    default:
	return ImageWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool ImageClickWidget::qt_emit( int _id, QUObject* _o )
{
    return ImageWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool ImageClickWidget::qt_property( int id, int f, QVariant* v)
{
    return ImageWidget::qt_property( id, f, v);
}

bool ImageClickWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
