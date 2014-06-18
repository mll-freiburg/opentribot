/****************************************************************************
** DeactivateWidget meta object code from reading C++ file 'DeactivateWidget.h'
**
** Created: Mon Aug 31 14:47:13 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "DeactivateWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *DeactivateWidget::className() const
{
    return "DeactivateWidget";
}

QMetaObject *DeactivateWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_DeactivateWidget( "DeactivateWidget", &DeactivateWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString DeactivateWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "DeactivateWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString DeactivateWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "DeactivateWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* DeactivateWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ 0, &static_QUType_ptr, "QPaintEvent", QUParameter::In }
    };
    static const QUMethod slot_0 = {"paintEvent", 1, param_slot_0 };
    static const QUParameter param_slot_1[] = {
	{ 0, &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_1 = {"keyPressEvent", 1, param_slot_1 };
    static const QUParameter param_slot_2[] = {
	{ 0, &static_QUType_ptr, "QCloseEvent", QUParameter::In }
    };
    static const QUMethod slot_2 = {"closeEvent", 1, param_slot_2 };
    static const QUParameter param_slot_3[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_3 = {"mousePressEvent", 1, param_slot_3 };
    static const QUParameter param_slot_4[] = {
	{ 0, &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_4 = {"mouseReleaseEvent", 1, param_slot_4 };
    static const QUMethod slot_5 = {"sendDeactivate", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "paintEvent(QPaintEvent*)", &slot_0, QMetaData::Protected },
	{ "keyPressEvent(QKeyEvent*)", &slot_1, QMetaData::Protected },
	{ "closeEvent(QCloseEvent*)", &slot_2, QMetaData::Protected },
	{ "mousePressEvent(QMouseEvent*)", &slot_3, QMetaData::Protected },
	{ "mouseReleaseEvent(QMouseEvent*)", &slot_4, QMetaData::Protected },
	{ "sendDeactivate()", &slot_5, QMetaData::Protected }
    };
    static const QUMethod signal_0 = {"clicked", 0, 0 };
    static const QMetaData signal_tbl[] = {
	{ "clicked()", &signal_0, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"DeactivateWidget", parentObject,
	slot_tbl, 6,
	signal_tbl, 1,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_DeactivateWidget.setMetaObject( metaObj );
    return metaObj;
}

void* DeactivateWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "DeactivateWidget" ) )
	return this;
    return QWidget::qt_cast( clname );
}

// SIGNAL clicked
void DeactivateWidget::clicked()
{
    activate_signal( staticMetaObject()->signalOffset() + 0 );
}

bool DeactivateWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: paintEvent((QPaintEvent*)static_QUType_ptr.get(_o+1)); break;
    case 1: keyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 2: closeEvent((QCloseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 3: mousePressEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 4: mouseReleaseEvent((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 5: sendDeactivate(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool DeactivateWidget::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: clicked(); break;
    default:
	return QWidget::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool DeactivateWidget::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool DeactivateWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
