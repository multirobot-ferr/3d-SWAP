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
#include <QPalette>

#include <qt_gcs/LogManager.h>

//---------------------------------------------------------------------------------------------------------------------
GCS::GCS(int _argc, char **_argv, QWidget *parent) : QMainWindow(parent) {
    LogManager::get()->status("MAIN_WINDOW", "Initializing GCS.");
    this->setWindowTitle("Ground Control Station");
    // Set up central widget and layout
    mCentralWidget = new QWidget();
    mMainLayout = new QHBoxLayout();
    mCentralWidget->setLayout(mMainLayout);
    this->setCentralWidget(mCentralWidget);

    // Set up  map
    mMapWidget= new Marble::MarbleWidget();
    mMapWidget->setProjection(Marble::Mercator);
    mMapWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    mMapWidget->show();
    mMainLayout->addWidget(mMapWidget);
    LogManager::get()->status("MAIN_WINDOW", "Initialized map.");

    // Set up uav list
    mUavListGroup = new QGroupBox();
    mUavListGroup->setTitle("UAV list");
    mUavListLayout = new QVBoxLayout();
    mUavListGroup->setLayout(mUavListLayout);
    mMainLayout->addWidget(mUavListGroup);

    mUavInterface1 = new UavInterface(_argc, _argv, 1, mMapWidget);
    LogManager::get()->status("MAIN_WINDOW", "Initializing UAV 1 interface.");
    mUavListLayout->addWidget(mUavInterface1);

    mUavInterface2 = new UavInterface(_argc, _argv, 2, mMapWidget);
    LogManager::get()->status("MAIN_WINDOW", "Initializing UAV 2 interface.");
    mUavListLayout->addWidget(mUavInterface2);

    mUavInterface3 = new UavInterface(_argc, _argv, 3, mMapWidget);
    LogManager::get()->status("MAIN_WINDOW", "Initializing UAV 3 interface.");
    mUavListLayout->addWidget(mUavInterface3);

    // Set color Background
    QPalette Pal(palette());
    Pal.setColor(QPalette::Background, Qt::gray);
    this->setAutoFillBackground(true);
    this->setPalette(Pal);
}

//---------------------------------------------------------------------------------------------------------------------
GCS::~GCS() {

}
