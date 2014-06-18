/****************************************************************************
** NewDialog meta object code from reading C++ file 'NewDialog.h'
**
** Created: Thu Aug 17 12:33:37 2006
**      by: The Qt MOC ($Id: moc_NewDialog.cpp,v 1.1.1.1 2007-05-04 14:20:26 tribots Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "NewDialog.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *NewDialog::className() const
{
    return "NewDialog";
}

QMetaObject *NewDialog::metaObj = 0;
static QMetaObjectCleanUp cleanUp_NewDialog( "NewDialog", &NewDialog::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString NewDialog::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "NewDialog", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString NewDialog::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "NewDialog", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* NewDialog::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QDialog::staticMetaObject();
    static const QUMethod slot_0 = {"openFileChooser", 0, 0 };
    static const QUMethod slot_1 = {"createProject", 0, 0 };
    static const QUMethod slot_2 = {"languageChange", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "openFileChooser()", &slot_0, QMetaData::Public },
	{ "createProject()", &slot_1, QMetaData::Public },
	{ "languageChange()", &slot_2, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"NewDialog", parentObject,
	slot_tbl, 3,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_NewDialog.setMetaObject( metaObj );
    return metaObj;
}

void* NewDialog::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "NewDialog" ) )
	return this;
    return QDialog::qt_cast( clname );
}

bool NewDialog::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: openFileChooser(); break;
    case 1: createProject(); break;
    case 2: languageChange(); break;
    default:
	return QDialog::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool NewDialog::qt_emit( int _id, QUObject* _o )
{
    return QDialog::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool NewDialog::qt_property( int id, int f, QVariant* v)
{
    return QDialog::qt_property( id, f, v);
}

bool NewDialog::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
