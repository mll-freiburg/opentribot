/****************************************************************************
** mainWidget meta object code from reading C++ file 'mainWidget.h'
**
** Created: Mon Aug 31 14:47:02 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "mainWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *mainWidget::className() const
{
    return "mainWidget";
}

QMetaObject *mainWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_mainWidget( "mainWidget", &mainWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString mainWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "mainWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString mainWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "mainWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* mainWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QMainWindow::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ "fileName", &static_QUType_QString, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"slotLoadMainConfig", 1, param_slot_0 };
    static const QUMethod slot_1 = {"slotFileOpen", 0, 0 };
    static const QUMethod slot_2 = {"chooseOpenFile", 0, 0 };
    static const QUParameter param_slot_3[] = {
	{ "lv", &static_QUType_ptr, "QListViewItem", QUParameter::In }
    };
    static const QUMethod slot_3 = {"slotClickedListViewItem", 1, param_slot_3 };
    static const QUMethod slot_4 = {"slotFileSave", 0, 0 };
    static const QUMethod slot_5 = {"aboutSlot", 0, 0 };
    static const QUMethod slot_6 = {"slotReloadMainConfig", 0, 0 };
    static const QUMethod slot_7 = {"slotAskForSave", 0, 0 };
    static const QUMethod slot_8 = {"commentLine", 0, 0 };
    static const QUMethod slot_9 = {"closeClicked", 0, 0 };
    static const QUMethod slot_10 = {"languageChange", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "slotLoadMainConfig(const QString&)", &slot_0, QMetaData::Public },
	{ "slotFileOpen()", &slot_1, QMetaData::Public },
	{ "chooseOpenFile()", &slot_2, QMetaData::Public },
	{ "slotClickedListViewItem(QListViewItem*)", &slot_3, QMetaData::Public },
	{ "slotFileSave()", &slot_4, QMetaData::Public },
	{ "aboutSlot()", &slot_5, QMetaData::Public },
	{ "slotReloadMainConfig()", &slot_6, QMetaData::Public },
	{ "slotAskForSave()", &slot_7, QMetaData::Public },
	{ "commentLine()", &slot_8, QMetaData::Public },
	{ "closeClicked()", &slot_9, QMetaData::Public },
	{ "languageChange()", &slot_10, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"mainWidget", parentObject,
	slot_tbl, 11,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_mainWidget.setMetaObject( metaObj );
    return metaObj;
}

void* mainWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "mainWidget" ) )
	return this;
    return QMainWindow::qt_cast( clname );
}

bool mainWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: slotLoadMainConfig((const QString&)static_QUType_QString.get(_o+1)); break;
    case 1: slotFileOpen(); break;
    case 2: chooseOpenFile(); break;
    case 3: slotClickedListViewItem((QListViewItem*)static_QUType_ptr.get(_o+1)); break;
    case 4: slotFileSave(); break;
    case 5: aboutSlot(); break;
    case 6: slotReloadMainConfig(); break;
    case 7: slotAskForSave(); break;
    case 8: commentLine(); break;
    case 9: closeClicked(); break;
    case 10: languageChange(); break;
    default:
	return QMainWindow::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool mainWidget::qt_emit( int _id, QUObject* _o )
{
    return QMainWindow::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool mainWidget::qt_property( int id, int f, QVariant* v)
{
    return QMainWindow::qt_property( id, f, v);
}

bool mainWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
