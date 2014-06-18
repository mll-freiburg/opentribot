/****************************************************************************
** TribotsTools::IDSlider meta object code from reading C++ file 'IDQObjects.h'
**
** Created: Mon Aug 31 14:44:27 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "IDQObjects.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *TribotsTools::IDSlider::className() const
{
    return "TribotsTools::IDSlider";
}

QMetaObject *TribotsTools::IDSlider::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsTools__IDSlider( "TribotsTools::IDSlider", &TribotsTools::IDSlider::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsTools::IDSlider::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::IDSlider", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsTools::IDSlider::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::IDSlider", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsTools::IDSlider::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QSlider::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"captureValueChanged", 1, param_slot_0 };
    static const QMetaData slot_tbl[] = {
	{ "captureValueChanged(int)", &slot_0, QMetaData::Protected }
    };
    static const QUParameter param_signal_0[] = {
	{ 0, &static_QUType_ptr, "unsigned int", QUParameter::In },
	{ 0, &static_QUType_ptr, "unsigned int", QUParameter::In }
    };
    static const QUMethod signal_0 = {"sliderChanged", 2, param_signal_0 };
    static const QMetaData signal_tbl[] = {
	{ "sliderChanged(unsigned int,unsigned int)", &signal_0, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsTools::IDSlider", parentObject,
	slot_tbl, 1,
	signal_tbl, 1,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsTools__IDSlider.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsTools::IDSlider::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsTools::IDSlider" ) )
	return this;
    return QSlider::qt_cast( clname );
}

#include <qobjectdefs.h>
#include <qsignalslotimp.h>

// SIGNAL sliderChanged
void TribotsTools::IDSlider::sliderChanged( unsigned int t0, unsigned int t1 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 0 );
    if ( !clist )
	return;
    QUObject o[3];
    static_QUType_ptr.set(o+1,&t0);
    static_QUType_ptr.set(o+2,&t1);
    activate_signal( clist, o );
}

bool TribotsTools::IDSlider::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: captureValueChanged((int)static_QUType_int.get(_o+1)); break;
    default:
	return QSlider::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsTools::IDSlider::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: sliderChanged((unsigned int)(*((unsigned int*)static_QUType_ptr.get(_o+1))),(unsigned int)(*((unsigned int*)static_QUType_ptr.get(_o+2)))); break;
    default:
	return QSlider::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool TribotsTools::IDSlider::qt_property( int id, int f, QVariant* v)
{
    return QSlider::qt_property( id, f, v);
}

bool TribotsTools::IDSlider::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES


const char *TribotsTools::IDSpinBox::className() const
{
    return "TribotsTools::IDSpinBox";
}

QMetaObject *TribotsTools::IDSpinBox::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsTools__IDSpinBox( "TribotsTools::IDSpinBox", &TribotsTools::IDSpinBox::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsTools::IDSpinBox::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::IDSpinBox", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsTools::IDSpinBox::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::IDSpinBox", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsTools::IDSpinBox::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QSpinBox::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"captureValueChanged", 1, param_slot_0 };
    static const QMetaData slot_tbl[] = {
	{ "captureValueChanged(int)", &slot_0, QMetaData::Protected }
    };
    static const QUParameter param_signal_0[] = {
	{ 0, &static_QUType_ptr, "unsigned int", QUParameter::In },
	{ 0, &static_QUType_ptr, "unsigned int", QUParameter::In }
    };
    static const QUMethod signal_0 = {"spinBoxChanged", 2, param_signal_0 };
    static const QMetaData signal_tbl[] = {
	{ "spinBoxChanged(unsigned int,unsigned int)", &signal_0, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsTools::IDSpinBox", parentObject,
	slot_tbl, 1,
	signal_tbl, 1,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsTools__IDSpinBox.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsTools::IDSpinBox::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsTools::IDSpinBox" ) )
	return this;
    return QSpinBox::qt_cast( clname );
}

// SIGNAL spinBoxChanged
void TribotsTools::IDSpinBox::spinBoxChanged( unsigned int t0, unsigned int t1 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 0 );
    if ( !clist )
	return;
    QUObject o[3];
    static_QUType_ptr.set(o+1,&t0);
    static_QUType_ptr.set(o+2,&t1);
    activate_signal( clist, o );
}

bool TribotsTools::IDSpinBox::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: captureValueChanged((int)static_QUType_int.get(_o+1)); break;
    default:
	return QSpinBox::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsTools::IDSpinBox::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: spinBoxChanged((unsigned int)(*((unsigned int*)static_QUType_ptr.get(_o+1))),(unsigned int)(*((unsigned int*)static_QUType_ptr.get(_o+2)))); break;
    default:
	return QSpinBox::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool TribotsTools::IDSpinBox::qt_property( int id, int f, QVariant* v)
{
    return QSpinBox::qt_property( id, f, v);
}

bool TribotsTools::IDSpinBox::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES


const char *TribotsTools::IDComboBox::className() const
{
    return "TribotsTools::IDComboBox";
}

QMetaObject *TribotsTools::IDComboBox::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsTools__IDComboBox( "TribotsTools::IDComboBox", &TribotsTools::IDComboBox::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsTools::IDComboBox::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::IDComboBox", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsTools::IDComboBox::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::IDComboBox", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsTools::IDComboBox::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QComboBox::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"captureValueChanged", 1, param_slot_0 };
    static const QMetaData slot_tbl[] = {
	{ "captureValueChanged(int)", &slot_0, QMetaData::Protected }
    };
    static const QUParameter param_signal_0[] = {
	{ 0, &static_QUType_ptr, "unsigned int", QUParameter::In },
	{ 0, &static_QUType_QString, 0, QUParameter::In }
    };
    static const QUMethod signal_0 = {"comboBoxChanged", 2, param_signal_0 };
    static const QMetaData signal_tbl[] = {
	{ "comboBoxChanged(unsigned int,const QString&)", &signal_0, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsTools::IDComboBox", parentObject,
	slot_tbl, 1,
	signal_tbl, 1,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsTools__IDComboBox.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsTools::IDComboBox::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsTools::IDComboBox" ) )
	return this;
    return QComboBox::qt_cast( clname );
}

// SIGNAL comboBoxChanged
void TribotsTools::IDComboBox::comboBoxChanged( unsigned int t0, const QString& t1 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 0 );
    if ( !clist )
	return;
    QUObject o[3];
    static_QUType_ptr.set(o+1,&t0);
    static_QUType_QString.set(o+2,t1);
    activate_signal( clist, o );
}

bool TribotsTools::IDComboBox::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: captureValueChanged((int)static_QUType_int.get(_o+1)); break;
    default:
	return QComboBox::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsTools::IDComboBox::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: comboBoxChanged((unsigned int)(*((unsigned int*)static_QUType_ptr.get(_o+1))),(const QString&)static_QUType_QString.get(_o+2)); break;
    default:
	return QComboBox::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool TribotsTools::IDComboBox::qt_property( int id, int f, QVariant* v)
{
    return QComboBox::qt_property( id, f, v);
}

bool TribotsTools::IDComboBox::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES


const char *TribotsTools::IDAction::className() const
{
    return "TribotsTools::IDAction";
}

QMetaObject *TribotsTools::IDAction::metaObj = 0;
static QMetaObjectCleanUp cleanUp_TribotsTools__IDAction( "TribotsTools::IDAction", &TribotsTools::IDAction::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString TribotsTools::IDAction::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::IDAction", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString TribotsTools::IDAction::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "TribotsTools::IDAction", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* TribotsTools::IDAction::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QAction::staticMetaObject();
    static const QUMethod slot_0 = {"captureActivated", 0, 0 };
    static const QUParameter param_slot_1[] = {
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod slot_1 = {"captureToggled", 1, param_slot_1 };
    static const QMetaData slot_tbl[] = {
	{ "captureActivated()", &slot_0, QMetaData::Protected },
	{ "captureToggled(bool)", &slot_1, QMetaData::Protected }
    };
    static const QUParameter param_signal_0[] = {
	{ 0, &static_QUType_ptr, "unsigned int", QUParameter::In }
    };
    static const QUMethod signal_0 = {"activated", 1, param_signal_0 };
    static const QUParameter param_signal_1[] = {
	{ 0, &static_QUType_ptr, "unsigned int", QUParameter::In },
	{ 0, &static_QUType_bool, 0, QUParameter::In }
    };
    static const QUMethod signal_1 = {"toggled", 2, param_signal_1 };
    static const QMetaData signal_tbl[] = {
	{ "activated(unsigned int)", &signal_0, QMetaData::Protected },
	{ "toggled(unsigned int,bool)", &signal_1, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"TribotsTools::IDAction", parentObject,
	slot_tbl, 2,
	signal_tbl, 2,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_TribotsTools__IDAction.setMetaObject( metaObj );
    return metaObj;
}

void* TribotsTools::IDAction::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "TribotsTools::IDAction" ) )
	return this;
    return QAction::qt_cast( clname );
}

// SIGNAL activated
void TribotsTools::IDAction::activated( unsigned int t0 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 0 );
    if ( !clist )
	return;
    QUObject o[2];
    static_QUType_ptr.set(o+1,&t0);
    activate_signal( clist, o );
}

// SIGNAL toggled
void TribotsTools::IDAction::toggled( unsigned int t0, bool t1 )
{
    if ( signalsBlocked() )
	return;
    QConnectionList *clist = receivers( staticMetaObject()->signalOffset() + 1 );
    if ( !clist )
	return;
    QUObject o[3];
    static_QUType_ptr.set(o+1,&t0);
    static_QUType_bool.set(o+2,t1);
    activate_signal( clist, o );
}

bool TribotsTools::IDAction::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: captureActivated(); break;
    case 1: captureToggled((bool)static_QUType_bool.get(_o+1)); break;
    default:
	return QAction::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool TribotsTools::IDAction::qt_emit( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->signalOffset() ) {
    case 0: activated((unsigned int)(*((unsigned int*)static_QUType_ptr.get(_o+1)))); break;
    case 1: toggled((unsigned int)(*((unsigned int*)static_QUType_ptr.get(_o+1))),(bool)static_QUType_bool.get(_o+2)); break;
    default:
	return QAction::qt_emit(_id,_o);
    }
    return TRUE;
}
#ifndef QT_NO_PROPERTIES

bool TribotsTools::IDAction::qt_property( int id, int f, QVariant* v)
{
    return QAction::qt_property( id, f, v);
}

bool TribotsTools::IDAction::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
