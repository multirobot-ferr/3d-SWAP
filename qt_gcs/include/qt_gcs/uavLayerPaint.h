///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MBZIRC_QTGCS_UAVLAYER_PAINT_H_
#define MBZIRC_QTGCS_UAVLAYER_PAINT_H_
#include <marble/MarbleWidget.h>
#include <marble/MarbleMap.h>
#include <marble/MarbleModel.h>
#include <marble/GeoPainter.h>
#include <marble/LayerInterface.h>

#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtGui/QKeyEvent>

class UavLayerPaint: public QObject, public Marble::LayerInterface {
    Q_OBJECT
public:
    // Constructor
    UavLayerPaint(Marble::MarbleWidget* widget);

    // Implemented from LayerInterface
    virtual QStringList renderPosition() const;

    // Implemented from LayerInterface
    virtual bool render( Marble::GeoPainter *painter, Marble::ViewportParams *viewport,
       const QString& renderPos = "NONE", Marble::GeoSceneLayer * layer = 0 );

    // Overriding QObject
    virtual bool eventFilter(QObject *obj, QEvent *event);

    void position(double _longitude, double _latitude);
private:
    Marble::MarbleWidget* m_widget;
    double mLatitude=0.0, mLongitude=0.0;

};


#endif  //    MBZIRC_QTGCS_UAVLAYER_PAINT_H_
