/****************************************************************************
** DistanceCalibrationCollectionWidget meta object code from reading C++ file 'DistanceCalibrationCollectionWidget.h'
**
** Created: Tue Dec 15 15:47:27 2009
**      by: The Qt MOC ($Id: qt/moc_yacc.cpp   3.3.8   edited Feb 2 14:59 $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "DistanceCalibrationCollectionWidget.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.8b. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *DistanceCalibrationCollectionWidget::className() const
{
    return "DistanceCalibrationCollectionWidget";
}

QMetaObject *DistanceCalibrationCollectionWidget::metaObj = 0;
static QMetaObjectCleanUp cleanUp_DistanceCalibrationCollectionWidget( "DistanceCalibrationCollectionWidget", &DistanceCalibrationCollectionWidget::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString DistanceCalibrationCollectionWidget::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "DistanceCalibrationCollectionWidget", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString DistanceCalibrationCollectionWidget::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "DistanceCalibrationCollectionWidget", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* DistanceCalibrationCollectionWidget::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QMainWindow::staticMetaObject();
    static const QUMethod slot_0 = {"init", 0, 0 };
    static const QUMethod slot_1 = {"destroy", 0, 0 };
    static const QUMethod slot_2 = {"loop", 0, 0 };
    static const QUMethod slot_3 = {"languageChange", 0, 0 };
    static const QUMethod slot_4 = {"setCenterActionClicked", 0, 0 };
    static const QUMethod slot_5 = {"setMaskAddActionClicked", 0, 0 };
    static const QUMethod slot_6 = {"setBlueActionClicked", 0, 0 };
    static const QUMethod slot_7 = {"setMaskSubActionClicked", 0, 0 };
    static const QUMethod slot_8 = {"setRedActionClicked", 0, 0 };
    static const QUMethod slot_9 = {"setDirectionActionClicked", 0, 0 };
    static const QUMethod slot_10 = {"setBalanceActionClicked", 0, 0 };
    static const QUMethod slot_11 = {"generateImageMaskStart", 0, 0 };
    static const QUMethod slot_12 = {"generateMarkerLogStart", 0, 0 };
    static const QUMethod slot_13 = {"stopActions", 0, 0 };
    static const QUParameter param_slot_14[] = {
	{ "ev", &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_14 = {"mouseInImagePressed", 1, param_slot_14 };
    static const QUParameter param_slot_15[] = {
	{ "ev", &static_QUType_ptr, "QMouseEvent", QUParameter::In }
    };
    static const QUMethod slot_15 = {"mouseInImageMoved", 1, param_slot_15 };
    static const QUMethod slot_16 = {"openMaskClicked", 0, 0 };
    static const QUMethod slot_17 = {"saveMaskClicked", 0, 0 };
    static const QUMethod slot_18 = {"saveMaskAsClicked", 0, 0 };
    static const QUMethod slot_19 = {"saveMarkerClicked", 0, 0 };
    static const QUMethod slot_20 = {"saveMarkerAsClicked", 0, 0 };
    static const QUMethod slot_21 = {"resetRedClicked", 0, 0 };
    static const QUMethod slot_22 = {"saveCenterClicked", 0, 0 };
    static const QUParameter param_slot_23[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_23 = {"recalculateMask", 1, param_slot_23 };
    static const QUMethod slot_24 = {"saveExitActionClicked", 0, 0 };
    static const QUMethod slot_25 = {"commandLineHelp", 0, 0 };
    static const QUParameter param_slot_26[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_26 = {"dilateMask", 1, param_slot_26 };
    static const QUMethod slot_27 = {"generateCenterBalanceArea", 0, 0 };
    static const QUMethod slot_28 = {"saveDistlinesClicked", 0, 0 };
    static const QUMethod slot_29 = {"saveDistlinesAsClicked", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "init()", &slot_0, QMetaData::Public },
	{ "destroy()", &slot_1, QMetaData::Public },
	{ "loop()", &slot_2, QMetaData::Public },
	{ "languageChange()", &slot_3, QMetaData::Protected },
	{ "setCenterActionClicked()", &slot_4, QMetaData::Protected },
	{ "setMaskAddActionClicked()", &slot_5, QMetaData::Protected },
	{ "setBlueActionClicked()", &slot_6, QMetaData::Protected },
	{ "setMaskSubActionClicked()", &slot_7, QMetaData::Protected },
	{ "setRedActionClicked()", &slot_8, QMetaData::Protected },
	{ "setDirectionActionClicked()", &slot_9, QMetaData::Protected },
	{ "setBalanceActionClicked()", &slot_10, QMetaData::Protected },
	{ "generateImageMaskStart()", &slot_11, QMetaData::Protected },
	{ "generateMarkerLogStart()", &slot_12, QMetaData::Protected },
	{ "stopActions()", &slot_13, QMetaData::Protected },
	{ "mouseInImagePressed(QMouseEvent*)", &slot_14, QMetaData::Protected },
	{ "mouseInImageMoved(QMouseEvent*)", &slot_15, QMetaData::Protected },
	{ "openMaskClicked()", &slot_16, QMetaData::Protected },
	{ "saveMaskClicked()", &slot_17, QMetaData::Protected },
	{ "saveMaskAsClicked()", &slot_18, QMetaData::Protected },
	{ "saveMarkerClicked()", &slot_19, QMetaData::Protected },
	{ "saveMarkerAsClicked()", &slot_20, QMetaData::Protected },
	{ "resetRedClicked()", &slot_21, QMetaData::Protected },
	{ "saveCenterClicked()", &slot_22, QMetaData::Protected },
	{ "recalculateMask(int)", &slot_23, QMetaData::Protected },
	{ "saveExitActionClicked()", &slot_24, QMetaData::Protected },
	{ "commandLineHelp()", &slot_25, QMetaData::Protected },
	{ "dilateMask(int)", &slot_26, QMetaData::Protected },
	{ "generateCenterBalanceArea()", &slot_27, QMetaData::Protected },
	{ "saveDistlinesClicked()", &slot_28, QMetaData::Protected },
	{ "saveDistlinesAsClicked()", &slot_29, QMetaData::Protected }
    };
    metaObj = QMetaObject::new_metaobject(
	"DistanceCalibrationCollectionWidget", parentObject,
	slot_tbl, 30,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_DistanceCalibrationCollectionWidget.setMetaObject( metaObj );
    return metaObj;
}

void* DistanceCalibrationCollectionWidget::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "DistanceCalibrationCollectionWidget" ) )
	return this;
    return QMainWindow::qt_cast( clname );
}

bool DistanceCalibrationCollectionWidget::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: init(); break;
    case 1: destroy(); break;
    case 2: loop(); break;
    case 3: languageChange(); break;
    case 4: setCenterActionClicked(); break;
    case 5: setMaskAddActionClicked(); break;
    case 6: setBlueActionClicked(); break;
    case 7: setMaskSubActionClicked(); break;
    case 8: setRedActionClicked(); break;
    case 9: setDirectionActionClicked(); break;
    case 10: setBalanceActionClicked(); break;
    case 11: generateImageMaskStart(); break;
    case 12: generateMarkerLogStart(); break;
    case 13: stopActions(); break;
    case 14: mouseInImagePressed((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 15: mouseInImageMoved((QMouseEvent*)static_QUType_ptr.get(_o+1)); break;
    case 16: openMaskClicked(); break;
    case 17: saveMaskClicked(); break;
    case 18: saveMaskAsClicked(); break;
    case 19: saveMarkerClicked(); break;
    case 20: saveMarkerAsClicked(); break;
    case 21: resetRedClicked(); break;
    case 22: saveCenterClicked(); break;
    case 23: recalculateMask((int)static_QUType_int.get(_o+1)); break;
    case 24: saveExitActionClicked(); break;
    case 25: commandLineHelp(); break;
    case 26: dilateMask((int)static_QUType_int.get(_o+1)); break;
    case 27: generateCenterBalanceArea(); break;
    case 28: saveDistlinesClicked(); break;
    case 29: saveDistlinesAsClicked(); break;
    default:
	return QMainWindow::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool DistanceCalibrationCollectionWidget::qt_emit( int _id, QUObject* _o )
{
    return QMainWindow::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool DistanceCalibrationCollectionWidget::qt_property( int id, int f, QVariant* v)
{
    return QMainWindow::qt_property( id, f, v);
}

bool DistanceCalibrationCollectionWidget::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
