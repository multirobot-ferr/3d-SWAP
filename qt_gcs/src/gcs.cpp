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

    // Set up interface UAV1
    mUavInterface1 = new QGroupBox();
    mUavInterface1->setTitle("UAV 1");
    QHBoxLayout *mainLayoutUav1 = new QHBoxLayout();
    mUavInterface1->setLayout(mainLayoutUav1);
    mUavListLayout->addWidget(mUavInterface1);

    QVBoxLayout *odometryLayoutUav1 = new QVBoxLayout();
    mainLayoutUav1->addLayout(odometryLayoutUav1);
    QLabel *altitudeText = new QLabel(tr("Altitude:  "));
    QLabel *latitudeText = new QLabel(tr("Latitude:  "));
    QLabel *longitudeText = new QLabel(tr("Longitude:  "));
    odometryLayoutUav1->addWidget(altitudeText);
    odometryLayoutUav1->addWidget(latitudeText);
    odometryLayoutUav1->addWidget(longitudeText);

    QVBoxLayout *actionsLayoutUav1 = new QVBoxLayout();
    mainLayoutUav1->addLayout(actionsLayoutUav1);

    QHBoxLayout *takeOffLayout1 = new QHBoxLayout();
    actionsLayoutUav1->addLayout(takeOffLayout1);
    QPushButton *takeOffButton = new QPushButton("Take off");
    takeOffLayout1->addWidget(takeOffButton);
    QSpinBox *takeOffAltitude = new QSpinBox();
    takeOffAltitude->setRange(0, 30);
    takeOffLayout1->addWidget(takeOffAltitude);

    QHBoxLayout *targetLayout1 = new QHBoxLayout();
    actionsLayoutUav1->addLayout(targetLayout1);
    QPushButton *targetButton1 = new QPushButton("Send target");
    targetLayout1->addWidget(targetButton1);
    QSpinBox *colorSpin1 = new QSpinBox();
    colorSpin1->setRange(-1, 2);
    QSpinBox *shapeSpin1 = new QSpinBox();
    shapeSpin1->setRange(-1, 2);
    targetLayout1->addWidget(colorSpin1);
    targetLayout1->addWidget(shapeSpin1);
}

//---------------------------------------------------------------------------------------------------------------------
GCS::~GCS() {

}
