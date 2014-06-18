/****************************************************************************
** topWindow meta object code from reading C++ file 'topwindow.h'
**
** Created: Mon Aug 31 14:47:40 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "topwindow.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *topWindow::className() const
{
    return "topWindow";
}

QMetaObject *topWindow::metaObj = 0;
static QMetaObjectCleanUp cleanUp_topWindow( "topWindow", &topWindow::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString topWindow::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "topWindow", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString topWindow::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "topWindow", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* topWindow::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUMethod slot_0 = {"open", 0, 0 };
    static const QUMethod slot_1 = {"save_marker", 0, 0 };
    static const QUMethod slot_2 = {"save_lines", 0, 0 };
    static const QUMethod slot_3 = {"save_markerDialog", 0, 0 };
    static const QUMethod slot_4 = {"save_lineDialog", 0, 0 };
    static const QUMethod slot_5 = {"generate_lines", 0, 0 };
    static const QUMethod slot_6 = {"showHelp", 0, 0 };
    static const QUMethod slot_7 = {"showSelection", 0, 0 };
    static const QUMethod slot_8 = {"about", 0, 0 };
    static const QUMethod slot_9 = {"terminateApplication", 0, 0 };
    static const QUMethod slot_10 = {"saveAndExit", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "open()", &slot_0, QMetaData::Public },
	{ "save_marker()", &slot_1, QMetaData::Public },
	{ "save_lines()", &slot_2, QMetaData::Public },
	{ "save_markerDialog()", &slot_3, QMetaData::Public },
	{ "save_lineDialog()", &slot_4, QMetaData::Public },
	{ "generate_lines()", &slot_5, QMetaData::Public },
	{ "showHelp()", &slot_6, QMetaData::Public },
	{ "showSelection()", &slot_7, QMetaData::Public },
	{ "about()", &slot_8, QMetaData::Public },
	{ "terminateApplication()", &slot_9, QMetaData::Public },
	{ "saveAndExit()", &slot_10, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"topWindow", parentObject,
	slot_tbl, 11,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_topWindow.setMetaObject( metaObj );
    return metaObj;
}

void* topWindow::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "topWindow" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool topWindow::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: open(); break;
    case 1: save_marker(); break;
    case 2: save_lines(); break;
    case 3: save_markerDialog(); break;
    case 4: save_lineDialog(); break;
    case 5: generate_lines(); break;
    case 6: showHelp(); break;
    case 7: showSelection(); break;
    case 8: about(); break;
    case 9: terminateApplication(); break;
    case 10: saveAndExit(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool topWindow::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool topWindow::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool topWindow::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
