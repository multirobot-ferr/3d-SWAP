///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "qt_gcs/uavLayerPaint.h"
#include "iostream"
using namespace Marble;

UavLayerPaint::UavLayerPaint(MarbleWidget* widget) : m_widget(widget) {
    // nothing to do
}

QStringList UavLayerPaint::renderPosition() const {
    // We will paint in exactly one of the following layers.
    // The current one can be changed by pressing the '+' key
    QStringList layers = QStringList() << "SURFACE" << "HOVERS_ABOVE_SURFACE";
    layers << "ORBIT" << "USER_TOOLS" << "STARS";

    return QStringList() << layers.at(0);
}

bool UavLayerPaint::eventFilter(QObject *obj, QEvent *event) {
    /*if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Plus) {
            ++m_index;
            return true;
        }
    }*/

    return false;
}

void UavLayerPaint::position(double _longitude, double _latitude) {
    mLatitude = _latitude;
    mLongitude = _longitude;
}

bool UavLayerPaint::render( GeoPainter *painter, ViewportParams *viewport, const QString& renderPos, GeoSceneLayer * layer ) {
    // Have window title reflect the current paint layer
    m_widget->setWindowTitle(renderPosition().first());
    GeoDataCoordinates home(mLongitude, mLatitude, 0.0, GeoDataCoordinates::Degree);

    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen( QPen(QBrush(QColor::fromRgb(255,255,255,200)), 3.0, Qt::SolidLine, Qt::RoundCap ) );
    painter->drawEllipse(home, 20, 20);
    return true;
}
