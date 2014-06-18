/****************************************************************************
** TribotsTools::ScrollImageWidget meta object code from reading C++ file 'ScrollImageWidget.h'
**
** Created: Tue Dec 15 15:47:26 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "ScrollImageWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *TribotsTools::ScrollImageWidget::className() const
{
    return "TribotsTools::ScrollImageWidget";
}

QMetaObject *TribotsTools::ScrollImageWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsTools__ScrollImageWidget( "TribotsTools::ScrollImageWidget", &TribotsTools::ScrollImageWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsTools::ScrollImageWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::ScrollImageWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsTools::ScrollImageWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::ScrollImageWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsTools::ScrollImageWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QScrollView::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ "new_image", &static_QUType_ptr, "Image", QUParameter::In }
    };
    static const QUMethod slot_0 = {"setImage", 1, param_slot_0 };
    static const QUParameter param_slot_1[] = {
	{ "new_image", &static_QUType_ptr, "ImageBuffer", QUParameter::In }
    };
    static const QUMethod slot_1 = {"setImage", 1, param_slot_1 };
    static const QUMethod slot_2 = {"centerImage", 0, 0 };
    static const QUParameter param_slot_3[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_3 = {"mousePressEvent", 1, param_slot_3 };
    static const QUParameter param_slot_4[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_4 = {"mouseMoveEvent", 1, param_slot_4 };
    static const QUParameter param_slot_5[] = {
	{ 0, &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_5 = {"keyPressEvent", 1, param_slot_5 };
    static const QMetaData slot_tbl[] = {
	{ "setImage(const Image&)", &slot_0, QMetaData::Public },
	{ "setImage(const ImageBuffer&)", &slot_1, QMetaData::Public },
	{ "centerImage()", &slot_2, QMetaData::Public },
	{ "mousePressEvent(QMouseEvent*)", &slot_3, QMetaData::Protected },
	{ "mouseMoveEvent(QMouseEvent*)", &slot_4, QMetaData::Protected },
	{ "keyPressEvent(QKeyEvent*)", &slot_5, QMetaData::Protected }
    };
    static const QUParameter param_signal_0[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod signal_0 = {"mousePressed", 1, param_signal_0 };
    static const QUParameter param_signal_1[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod signal_1 = {"mouseMoved", 1, param_signal_1 };
    static const QUParameter param_signal_2[] = {
	{ 0, &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod signal_2 = {"keyPressed", 1, param_signal_2 };
    static const QMetaData signal_tbl[] = {
	{ "mousePressed(QMouseEvent*)", &signal_0, QMetaData::Public },
	{ "mouseMoved(QMouseEvent*)", &signal_1, QMetaData::Public },
	{ "keyPressed(QKeyEvent*)", &signal_2, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsTools::ScrollImageWidget", parentObject,
	slot_tbl, 6,
	signal_tbl, 3,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsTools__ScrollImageWidget.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsTools::ScrollImageWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsTools::ScrollImageWidget" ) )
	return this;
    return QScrollView::qt_cast( clname );
}

#include <qobjectdefs.h>
#include <qsignalslotimp.h>

// SIGNAL mousePressed
void TribotsTools::ScrollImageWidget::mousePressed( QMouseEvent* t0 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 0 );
    if ( !clist )
	return;
    QUObject o[2];
    static_QUType_ptr.set(o+1,t0);
    activate_signal( clist, o );
}

// SIGNAL mouseMoved
void TribotsTools::ScrollImageWidget::mouseMoved( QMouseEvent* t0 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 1 );
    if ( !clist )
	return;
    QUObject o[2];
    static_QUType_ptr.set(o+1,t0);
    activate_signal( clist, o );
}

// SIGNAL keyPressed
void TribotsTools::ScrollImageWidget::keyPressed( QKeyEvent* t0 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 2 );
    if ( !clist )
	return;
    QUObject o[2];
    static_QUType_ptr.set(o+1,t0);
    activate_signal( clist, o );
}

bool TribotsTools::ScrollImageWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: setImage((const Image&)*((const Image*)static_QUType_ptr.get(_o+1))); break;
    case 1: setImage((const ImageBuffer&)*((const ImageBuffer*)static_QUType_ptr.get(_o+1))); break;
    case 2: centerImage(); break;
    case 3: mousePressEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 4: mouseMoveEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 5: keyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    default:
	return QScrollView::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsTools::ScrollImageWidget::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: mousePressed((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 1: mouseMoved((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 2: keyPressed((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    default:
	return QScrollView::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool TribotsTools::ScrollImageWidget::qt_property( int id, int f, QVariant* v)
{
    return QScrollView::qt_property( id, f, v);
}

bool TribotsTools::ScrollImageWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
