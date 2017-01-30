///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "qt_gcs/UavMark.h"

#include <marble/MarbleModel.h>
#include <marble/GeoDataTreeModel.h>

#include <iostream>

UavMark::UavMark(Marble::MarbleWidget *mainMapWidget, std::string _id):mMainMapWidget(mainMapWidget) {
    mDocument = new Marble::GeoDataDocument();
    mMark = new Marble::GeoDataPlacemark(_id.c_str());
    mDocument->append(mMark);
    mMainMapWidget->model()->treeModel()->addDocument(mDocument);

    connect(this, SIGNAL(coordinatesChanged(Marble::GeoDataCoordinates)),
                this, SLOT(setUavCoordinates(Marble::GeoDataCoordinates)), Qt::BlockingQueuedConnection);

    mLastTimeUpdated = std::chrono::system_clock::now();
}

void UavMark::newPosition(double _longitude, double _latitude) {
    auto coordinates = Marble::GeoDataCoordinates(_longitude, _latitude, 0, Marble::GeoDataCoordinates::Degree);
    mCoodinatesMutex.lock();
    mLastCoordinate = coordinates;
    mCoodinatesMutex.unlock();
    int elapsedTime= std::chrono::duration_cast<std::chrono::milliseconds> ( std::chrono::system_clock::now()-mLastTimeUpdated).count();
    if(elapsedTime>100){
        emit coordinatesChanged(coordinates);
        mLastTimeUpdated = std::chrono::system_clock::now();
    }

}

void UavMark::position(double &_longitude, double &_latitude) {
    mCoodinatesMutex.lock();
    _longitude = mLastCoordinate.longitude();
    _latitude = mLastCoordinate.latitude();
    mCoodinatesMutex.unlock();

}

void UavMark::setUavCoordinates(const Marble::GeoDataCoordinates &coord) {
    mMark->setCoordinate(coord);
    mMainMapWidget->model()->treeModel()->updateFeature(mMark);
}
