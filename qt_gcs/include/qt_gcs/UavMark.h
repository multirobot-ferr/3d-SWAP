///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _QTGCS_UAVMARK_H_
#define _QTGCS_UAVMARK_H_

#include <QObject>

#include <marble/MarbleWidget.h>
#include <marble/GeoDataCoordinates.h>
#include <marble/GeoDataDocument.h>
#include <marble/GeoDataPlacemark.h>

#include <string>
#include <mutex>
class UavMark : public QObject{
    Q_OBJECT

public:
    UavMark(Marble::MarbleWidget *mainMapWidget, std::__cxx11::string _id);
    void newPosition(double _longitude, double _latitude);
    void position(double &_longitude, double &_latitude);

signals:
    void coordinatesChanged(Marble::GeoDataCoordinates coord);

public slots:
    void setUavCoordinates(const Marble::GeoDataCoordinates &coord);

private:
    std::mutex mCoodinatesMutex;
    Marble::GeoDataCoordinates mLastCoordinate;
    qreal mAlpha;

    Marble::GeoDataDocument *mDocument;
    Marble::GeoDataPlacemark *mMark;

    Marble::MarbleWidget *mMainMapWidget;
};

#endif
