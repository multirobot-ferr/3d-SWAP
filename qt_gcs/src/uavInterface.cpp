///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "qt_gcs/uavInterface.h"

#include <QLabel>
#include <string>
#include <chrono>
#include <thread>

#include <grvc_utils/argument_parser.h>

#include <uav_visual_servoing/target_service.h>
#include <uav_visual_servoing/takeoff_service.h>

#include <ros/ros.h>

//---------------------------------------------------------------------------------------------------------------------
UavInterface::UavInterface(int _argc, char** _argv, int _index) {
    mUavId = _index;
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
    mTakeOffAltitude = new QDoubleSpinBox();
    mTakeOffAltitude->setRange(1, 30);
    mTakeOffLayout->addWidget(mTakeOffAltitude);

    mTargetLayout = new QHBoxLayout();
    mActionsLayoutUav->addLayout(mTargetLayout);
    mTargetButton = new QPushButton("Send target");
    mTargetLayout->addWidget(mTargetButton);
    mColorSpin = new QSpinBox();
    mColorSpin->setRange(-1, 2);
    mShapeSpin = new QSpinBox();
    mShapeSpin->setRange(-1, 2);
    mTargetEnable = new QRadioButton();
    mTargetLayout->addWidget(mTargetEnable);
    mTargetLayout->addWidget(mColorSpin);
    mTargetLayout->addWidget(mShapeSpin);

    // Set callbacks
    connect(mTakeOffButton, SIGNAL (released()), this, SLOT (takeOffCallback()));
    connect(mTargetButton, SIGNAL (released()), this, SLOT (targetCallback()));
}

//---------------------------------------------------------------------------------------------------------------------
UavInterface::~UavInterface() {

}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::takeOffCallback(){
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<uav_visual_servoing::takeoff_service>("/mbzirc_"+std::to_string(mUavId)+"/visual_servoing/takeoff");
    uav_visual_servoing::takeoff_service call;
    call.request.altitude = mTakeOffAltitude->value();
    client.call(call);
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::targetCallback(){
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<uav_visual_servoing::target_service>("/mbzirc_"+std::to_string(mUavId)+"/visual_servoing/enabled");
    uav_visual_servoing::target_service call;
    call.request.enabled = mTargetEnable->isChecked();
    call.request.color = mColorSpin->value();
    call.request.shape = mShapeSpin->value();
    client.call(call);
}
