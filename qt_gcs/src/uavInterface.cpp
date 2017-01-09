///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "qt_gcs/uavInterface.h"

#include <QLabel>
#include <string>

//---------------------------------------------------------------------------------------------------------------------
UavInterface::UavInterface(int _index) {
    std::string title = "UAV " + std::to_string(_index);
    this->setTitle(title.c_str());
    mMainLayoutUav = new QHBoxLayout();
    this->setLayout(mMainLayoutUav);

    mOdometryLayoutUav = new QVBoxLayout();
    mMainLayoutUav->addLayout(mOdometryLayoutUav);
    QLabel *altitudeText = new QLabel(tr("Altitude:  "));
    QLabel *latitudeText = new QLabel(tr("Latitude:  "));
    QLabel *longitudeText = new QLabel(tr("Longitude:  "));
    mOdometryLayoutUav->addWidget(altitudeText);
    mOdometryLayoutUav->addWidget(latitudeText);
    mOdometryLayoutUav->addWidget(longitudeText);

    mActionsLayoutUav = new QVBoxLayout();
    mMainLayoutUav->addLayout(mActionsLayoutUav);

    mTakeOffLayout = new QHBoxLayout();
    mActionsLayoutUav->addLayout(mTakeOffLayout);
    mTakeOffButton = new QPushButton("Take off");
    mTakeOffLayout->addWidget(mTakeOffButton);
    mTakeOffAltitude = new QSpinBox();
    mTakeOffAltitude->setRange(0, 30);
    mTakeOffLayout->addWidget(mTakeOffAltitude);

    mTargetLayout = new QHBoxLayout();
    mActionsLayoutUav->addLayout(mTargetLayout);
    mTargetButton = new QPushButton("Send target");
    mTargetLayout->addWidget(mTargetButton);
    mColorSpin = new QSpinBox();
    mColorSpin->setRange(-1, 2);
    mShapeSpin = new QSpinBox();
    mShapeSpin->setRange(-1, 2);
    mTargetLayout->addWidget(mColorSpin);
    mTargetLayout->addWidget(mShapeSpin);
}

//---------------------------------------------------------------------------------------------------------------------
UavInterface::~UavInterface() {

}
