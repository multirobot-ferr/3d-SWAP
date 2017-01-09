///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "qt_gcs/gcs.h"
#include <QTextBlock>
#include <QSpinBox>
#include <QPushButton>
#include <QSpinBox>

//---------------------------------------------------------------------------------------------------------------------
GCS::GCS(QWidget *parent) : QMainWindow(parent) {
    this->setWindowTitle("Ground Control Station");
    // Set up central widget and layout
    mCentralWidget = new QWidget();
    mMainLayout = new QHBoxLayout();
    mCentralWidget->setLayout(mMainLayout);
    this->setCentralWidget(mCentralWidget);

    // Set up dummy map
    mDummyMap = new QImage();
    mDummyMap->load("/home/bardo91/Desktop/dummyMap.png");
    mMapLayout = new QLabel();
    mMapLayout->setPixmap(QPixmap::fromImage(*mDummyMap));
    mMainLayout->addWidget(mMapLayout);

    // Set up uav list
    mUavListGroup = new QGroupBox();
    mUavListGroup->setTitle("UAV list");
    mUavListLayout = new QVBoxLayout();
    mUavListGroup->setLayout(mUavListLayout);
    mMainLayout->addWidget(mUavListGroup);

    mUavInterface1 = new UavInterface(1);
    mUavListLayout->addWidget(mUavInterface1);
    mUavInterface2 = new UavInterface(2);
    mUavListLayout->addWidget(mUavInterface2);
    mUavInterface3 = new UavInterface(3);
    mUavListLayout->addWidget(mUavInterface3);
}

//---------------------------------------------------------------------------------------------------------------------
GCS::~GCS() {

}
