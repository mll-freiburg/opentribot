/****************************************************************************
** MainWindow meta object code from reading C++ file 'MainWindow.h'
**
** Created: Thu Aug 17 14:46:44 2006
**      by: The Qt MOC ($Id: moc_MainWindow.cpp,v 1.1.1.1 2007-05-04 14:20:26 tribots Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "MainWindow.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *MainWindow::className() const
{
    return "MainWindow";
}

QMetaObject *MainWindow::metaObj = 0;
static QMetaObjectCleanUp cleanUp_MainWindow( "MainWindow", &MainWindow::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString MainWindow::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MainWindow", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString MainWindow::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MainWindow", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* MainWindow::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QMainWindow::staticMetaObject();
    static const QUMethod slot_0 = {"fileNewProject", 0, 0 };
    static const QUMethod slot_1 = {"fileOpenProject", 0, 0 };
    static const QUMethod slot_2 = {"fileSaveProject", 0, 0 };
    static const QUMethod slot_3 = {"fileCloseProject", 0, 0 };
    static const QUMethod slot_4 = {"fileExitProject", 0, 0 };
    static const QUMethod slot_5 = {"projectAddPicture", 0, 0 };
    static const QUMethod slot_6 = {"toggleModelView", 0, 0 };
    static const QUParameter param_slot_7[] = {
	{ "action", &static_QUType_ptr, "QAction", QUParameter::In }
    };
    static const QUMethod slot_7 = {"changeProject", 1, param_slot_7 };
    static const QUParameter param_slot_8[] = {
	{ "e", &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_8 = {"keyPressEvent", 1, param_slot_8 };
    static const QUParameter param_slot_9[] = {
	{ "e", &static_QUType_ptr, "QKeyEvent", QUParameter::In }
    };
    static const QUMethod slot_9 = {"keyReleaseEvent", 1, param_slot_9 };
    static const QUMethod slot_10 = {"languageChange", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "fileNewProject()", &slot_0, QMetaData::Public },
	{ "fileOpenProject()", &slot_1, QMetaData::Public },
	{ "fileSaveProject()", &slot_2, QMetaData::Public },
	{ "fileCloseProject()", &slot_3, QMetaData::Public },
	{ "fileExitProject()", &slot_4, QMetaData::Public },
	{ "projectAddPicture()", &slot_5, QMetaData::Public },
	{ "toggleModelView()", &slot_6, QMetaData::Public },
	{ "changeProject(QAction*)", &slot_7, QMetaData::Public },
	{ "keyPressEvent(QKeyEvent*)", &slot_8, QMetaData::Public },
	{ "keyReleaseEvent(QKeyEvent*)", &slot_9, QMetaData::Public },
	{ "languageChange()", &slot_10, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"MainWindow", parentObject,
	slot_tbl, 11,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_MainWindow.setMetaObject( metaObj );
    return metaObj;
}

void* MainWindow::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "MainWindow" ) )
	return this;
    return QMainWindow::qt_cast( clname );
}

bool MainWindow::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: fileNewProject(); break;
    case 1: fileOpenProject(); break;
    case 2: fileSaveProject(); break;
    case 3: fileCloseProject(); break;
    case 4: fileExitProject(); break;
    case 5: projectAddPicture(); break;
    case 6: toggleModelView(); break;
    case 7: changeProject((QAction*)static_QUType_ptr.get(_o+1)); break;
    case 8: keyPressEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 9: keyReleaseEvent((QKeyEvent*)static_QUType_ptr.get(_o+1)); break;
    case 10: languageChange(); break;
    default:
	return QMainWindow::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool MainWindow::qt_emit( int _id, QUObject* _o )
{
    return QMainWindow::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool MainWindow::qt_property( int id, int f, QVariant* v)
{
    return QMainWindow::qt_property( id, f, v);
}

bool MainWindow::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
