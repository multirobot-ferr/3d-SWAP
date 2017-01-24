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
#include <uav_visual_servoing/land_service.h>

#include <ros/ros.h>

#include <mavros_msgs/ActuatorControl.h>

#include <qt_gcs/LogManager.h>

//---------------------------------------------------------------------------------------------------------------------
UavInterface::UavInterface(int _argc, char** _argv, int _index, Marble::MarbleWidget *_mapPtr) {
    mUavId = _index;
    mMapPtr = _mapPtr;

    // Main config
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Initializing UAV interface.");
    std::string title = "UAV " + std::to_string(_index);
    this->setTitle(title.c_str());
    mMainLayoutUav = new QHBoxLayout();
    this->setLayout(mMainLayoutUav);

    mOdometryLayoutUav = new QVBoxLayout();
    mMainLayoutUav->addLayout(mOdometryLayoutUav);

    // Odometry altitude
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
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Initialized subscription to altitude.");

    // Odometri geodesics
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
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Initialized subscription to geodesic position.");

    // Actions
    mActionsLayoutUav = new QVBoxLayout();
    mMainLayoutUav->addLayout(mActionsLayoutUav);

    // Center target on map
    mCenterTarget = new QPushButton("Center target");
    mActionsLayoutUav->addWidget(mCenterTarget);

    // Take off panel
    mTakeOffLayout = new QHBoxLayout();
    mActionsLayoutUav->addLayout(mTakeOffLayout);
    mTakeOffButton = new QPushButton("Take off");
    mTakeOffLayout->addWidget(mTakeOffButton);
    mTakeOffAltitude = new QDoubleSpinBox();
    mTakeOffAltitude->setRange(1, 30);
    mTakeOffLayout->addWidget(mTakeOffAltitude);

    // Land panel
    mLandLayout = new QHBoxLayout();
    mActionsLayoutUav->addLayout(mLandLayout);
    mLandButton = new QPushButton("Land");
    mLandLayout->addWidget(mLandButton);


    // Target panel
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

    // Magnet
    QHBoxLayout *mMagnetLayout = new QHBoxLayout();
    mToggleMagnet = new QPushButton("Switch magnet");
    mToggleMagnet->setCheckable(true);
    mMagnetLayout->addWidget(mToggleMagnet);
    mMagnetLed = new LedIndicator();
    mMagnetLed->setState(false);
    mMagnetLayout->addWidget(mMagnetLed);
    mActionsLayoutUav->addLayout(mMagnetLayout);
    mMagnetSubscriber = nh.subscribe("/mavros_"+std::to_string(_index)+"/rc/out",
                                     1,
                                     &UavInterface::magnetInterruptorCallback,this);

    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Initialized action buttons.");

    // Set callbacks
    connect(mTakeOffButton, SIGNAL (released()), this, SLOT (takeOffCallback()));
    connect(mLandButton, SIGNAL (released()), this, SLOT (landCallback()));
    connect(mTargetButton, SIGNAL (released()), this, SLOT (targetCallback()));
    connect(mCenterTarget, SIGNAL (released()), this, SLOT (centerCallback()));
    connect(mToggleMagnet, SIGNAL (toggled(bool)), this, SLOT (switchMagnetCallback(bool)));
    connect(mToggleMagnet, SIGNAL (toggled(bool)), this, SLOT (switchMagnetCallback(bool)));
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Initialized connected callbacks of action buttons.");

    // Add visualization on map
    mUavMark = new UavMark(_mapPtr, "Uav_"+std::to_string(mUavId));
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Created mark on map.");
}

//---------------------------------------------------------------------------------------------------------------------
UavInterface::~UavInterface() {

}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::takeOffCallback(){
    mTakeOffButton->setEnabled(false);
    mLandButton->setEnabled(false);
    mTakeOffThread = new std::thread([this](){
        ros::NodeHandle nh;
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Sending take off service.");
        ros::ServiceClient client = nh.serviceClient<uav_visual_servoing::takeoff_service>("/mbzirc_"+std::to_string(mUavId)+"/visual_servoing/takeoff");
        uav_visual_servoing::takeoff_service call;
        call.request.altitude = mTakeOffAltitude->value();
        auto res = client.call(call);
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Returned call with result: "+std::to_string(res));
        mTakeOffButton->setEnabled(true);
        mLandButton->setEnabled(true);
    });

}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::landCallback(){
    mLandButton->setEnabled(false);
    mTakeOffButton->setEnabled(false);
    mLandThread = new std::thread([this](){
        ros::NodeHandle nh;
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Sending land service.");
        ros::ServiceClient client = nh.serviceClient<uav_visual_servoing::land_service>("/mbzirc_"+std::to_string(mUavId)+"/visual_servoing/land");
        uav_visual_servoing::land_service call;
        auto res = client.call(call);
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Returned call with result: "+std::to_string(res));
        mLandButton->setEnabled(true);
        mTakeOffButton->setEnabled(true);
    });

}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::targetCallback(){
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<uav_visual_servoing::target_service>("/mbzirc_"+std::to_string(mUavId)+"/visual_servoing/enabled");
    uav_visual_servoing::target_service call;
    call.request.enabled = mTargetEnable->isChecked();
    call.request.color = mColorSpin->value();
    call.request.shape = mShapeSpin->value();
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Sending new target to UAV. Color: "+std::to_string(mColorSpin->value()));
    auto res = client.call(call);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Returned call with result: "+std::to_string(res));
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::centerCallback() {
    double longitude, latitude;
    mUavMark->position(longitude, latitude);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Centering map on UAV. latitude: " +std::to_string(latitude) + "and longitude: "+std::to_string(longitude));
    mMapPtr->centerOn(Marble::GeoDataCoordinates(longitude, latitude));
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::switchMagnetCallback(bool _state) {
    mToggleMagnet->setEnabled(false);
    mToggleMagnetThread = new std::thread([this, _state](){
        ros::NodeHandle nh;
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Switching magnet.");
        ros::Publisher magnetPublisher = nh.advertise<mavros_msgs::ActuatorControl>("/mavros_"+std::to_string(mUavId)+"/actuator_control",1);
        mavros_msgs::ActuatorControl controlSignal;
        controlSignal.group_mix = 3;
        controlSignal.controls[6] = (_state ? 1.0 : -1.0);

        // Start magnetization/demagnetization
        std::chrono::time_point<std::chrono::steady_clock> t0 = std::chrono::steady_clock::now();
        magnetPublisher.publish(controlSignal);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // Ensure that the magnet is resting
        controlSignal.controls[6] = 0.0;
        magnetPublisher.publish(controlSignal);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        mToggleMagnet->setEnabled(true);
    });

}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::altitudeCallback(const std_msgs::Float64ConstPtr &_msg) {
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updating display altitude: " + std::to_string(_msg->data));
    mAltitudeBox->display(_msg->data);
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::geodesicCallback(const sensor_msgs::NavSatFixConstPtr &_msg) {
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updating display longitude: " + std::to_string(_msg->longitude));
    mLongitudeBox->display(_msg->longitude);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updating display latitude: " + std::to_string(_msg->latitude));
    mLatitudeBox->display(_msg->latitude);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updating position in map");
    mUavMark->newPosition(_msg->longitude, _msg->latitude);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updated position in map");
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::magnetInterruptorCallback(const mavros_msgs::RCOutConstPtr &_msg) {
    if(_msg->channels[8] < 1100)
        mMagnetLed->setState(false);
    else if(_msg->channels[8] > 1900)
        mMagnetLed->setState(true);
}
