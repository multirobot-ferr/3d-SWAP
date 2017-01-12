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
UavInterface::UavInterface(int _argc, char** _argv, int _index, Marble::MarbleWidget *_mapPtr) {
    mUavId = _index;
    mMapPtr = _mapPtr;

    std::string title = "UAV " + std::to_string(_index);
    this->setTitle(title.c_str());
    mMainLayoutUav = new QHBoxLayout();
    this->setLayout(mMainLayoutUav);

    mOdometryLayoutUav = new QVBoxLayout();
    mMainLayoutUav->addLayout(mOdometryLayoutUav);

    QHBoxLayout *altitudeLayout = new QHBoxLayout();
    QLabel *altitudeText = new QLabel(tr("Altitude:  "));
    mAltitudeBox = new QLCDNumber();
    mAltitudeBox->setMode(QLCDNumber::Dec);
    mAltitudeBox->display(0.0);
    altitudeLayout->addWidget(altitudeText);
    altitudeLayout->addWidget(mAltitudeBox);
    mOdometryLayoutUav->addLayout(altitudeLayout);
    ros::NodeHandle nh;
    mAltitudeSubscriber = nh.subscribe("/mavros_"+std::to_string(_index)+"/global_position/rel_alt",
                                       1,
                                       &UavInterface::altitudeCallback,this);

    QHBoxLayout *latitudeLayout = new QHBoxLayout();
    QLabel *latitudeText = new QLabel(tr("Latitude:  "));
    mLatitudeBox  = new QLCDNumber();
    mLatitudeBox->setMode(QLCDNumber::Dec);
    mLatitudeBox->display(0.0);
    mLatitudeBox->setDigitCount(8);
    latitudeLayout->addWidget(latitudeText);
    latitudeLayout->addWidget(mLatitudeBox);
    mOdometryLayoutUav->addLayout(latitudeLayout);

    QHBoxLayout *longitudeLayout = new QHBoxLayout();
    QLabel *longitudeText = new QLabel(tr("Longitude:  "));
    mLongitudeBox = new QLCDNumber();
    mLongitudeBox->setMode(QLCDNumber::Dec);
    mLongitudeBox->display(0.0);
    mLongitudeBox->setDigitCount(8);
    longitudeLayout->addWidget(longitudeText);
    longitudeLayout->addWidget(mLongitudeBox);
    mOdometryLayoutUav->addLayout(longitudeLayout);
    mGeodesicSubscriber = nh.subscribe("/mavros_"+std::to_string(_index)+"/global_position/global",
                                       1,
                                       &UavInterface::geodesicCallback,this);

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


    // Add visualization on map
    mUavMark = new UavMark(_mapPtr, "Uav_"+std::to_string(mUavId));
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

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::altitudeCallback(const std_msgs::Float64ConstPtr &_msg) {
    mAltitudeBox->display(_msg->data);
}

void UavInterface::geodesicCallback(const sensor_msgs::NavSatFixConstPtr &_msg) {
    mLongitudeBox->display(_msg->longitude);
    mLatitudeBox->display(_msg->latitude);
    mUavMark->newPosition(_msg->longitude, _msg->latitude);
}
