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
#include<sstream>

#include <grvc_utils/argument_parser.h>

#include <uav_state_machine/target_service.h>
#include <uav_state_machine/takeoff_service.h>
#include <uav_state_machine/land_service.h>
#include <uav_state_machine/magnetize_service.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

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

    mHalPoseSubscriber = nh.subscribe("/mbzirc_"+std::to_string(_index)+"/hal/pose",
                                       1,
                                       &UavInterface::halPoseCallback,this);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Initialized subscription to hal pose.");

    // Actions
    mActionsLayoutUav = new QVBoxLayout();
    mMainLayoutUav->addLayout(mActionsLayoutUav);

    // Waypoints
    mWaypointListBox = new QComboBox();
    mWpAction = new QSpinBox();
    mWpAction->setRange(0,2);
    mSendWpList = new QPushButton("Send");
    mWaypointListLayout.addWidget(mWaypointListBox);
    mWaypointListLayout.addWidget(mWpAction);
    mWaypointListLayout.addWidget(mSendWpList);
    mWaypointLayout.addLayout(&mWaypointListLayout);

    mAddWpButton = new QPushButton("Add");
    mWpX = new QLineEdit();
    mWpY = new QLineEdit();
    mWpZ = new QLineEdit();
    mWaypointEditLayout.addWidget(mWpX);
    mWaypointEditLayout.addWidget(mWpY);
    mWaypointEditLayout.addWidget(mWpZ);
    mWaypointEditLayout.addWidget(mAddWpButton);
    mEraseWpButton = new QPushButton("Erase");
    mWaypointListLayout.addWidget(mEraseWpButton);
    mWaypointLayout.addLayout(&mWaypointEditLayout);
    mActionsLayoutUav->addLayout(&mWaypointLayout);

    // Center target on map
    //mCenterTarget = new QPushButton("Center target");
    //mActionsLayoutUav->addWidget(mCenterTarget);

    // Take off panel
    mTakeOffLayout = new QHBoxLayout();
    mActionsLayoutUav->addLayout(mTakeOffLayout);
    mTakeOffButton = new QPushButton("Take off");
    mTakeOffLayout->addWidget(mTakeOffButton);
    mTakeOffAltitude = new QDoubleSpinBox();
    mTakeOffAltitude->setRange(1, 30);
    mTakeOffLayout->addWidget(mTakeOffAltitude);

    // Land panel
    //mLandLayout = new QHBoxLayout();
    //mActionsLayoutUav->addLayout(mLandLayout);
    mLandButton = new QPushButton("Land");
    mTakeOffLayout->addWidget(mLandButton);
    //mLandLayout->addWidget(mLandButton);


    // Target panel
    mTargetLayout = new QHBoxLayout();
    mActionsLayoutUav->addLayout(mTargetLayout);
    mTargetButton = new QPushButton("Send target");
    mTargetLayout->addWidget(mTargetButton);
    mColorSpin = new QSpinBox();
    mColorSpin->setRange(-1, 4);
    //mShapeSpin = new QSpinBox();
    //mShapeSpin->setRange(-1, 4);
    mTargetEnable = new QRadioButton();
    mTargetLayout->addWidget(mTargetEnable);
    mTargetLayout->addWidget(mColorSpin);
    //mTargetLayout->addWidget(mShapeSpin);

    // Magnet
    QHBoxLayout *mMagnetLayout = new QHBoxLayout();
    mToggleMagnet = new QPushButton("Switch magnet");
    mToggleMagnet->setCheckable(true);
    mMagnetLayout->addWidget(mToggleMagnet);
    mInterruptorLed = new LedIndicator();
    mInterruptorLed->setState(false);
    mMagnetLayout->addWidget(mInterruptorLed);
    mActionsLayoutUav->addLayout(mMagnetLayout);
    mMagnetSubscriber = nh.subscribe("/mbzirc_"+std::to_string(_index)+"/catching_device/switch",
                                     1,
                                     &UavInterface::rcMagnetInterruptorCallback,this);

    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Initialized action buttons.");

    // Set callbacks
    connect(mTakeOffButton, SIGNAL (released()), this, SLOT (takeOffCallback()));
    connect(mLandButton, SIGNAL (released()), this, SLOT (landCallback()));
    connect(mTargetButton, SIGNAL (released()), this, SLOT (targetCallback()));
    connect(mAddWpButton, SIGNAL (released()), this, SLOT (addWpButtonCallback()));
    connect(mEraseWpButton, SIGNAL (released()), this, SLOT (eraseWpButtonCallback()));
    connect(mSendWpList, SIGNAL (released()), this, SLOT (sendWpButtonCallback()));
    //connect(mCenterTarget, SIGNAL (released()), this, SLOT (centerCallback()));
    connect(mToggleMagnet, SIGNAL (toggled(bool)), this, SLOT (switchMagnetCallback(bool)));
    connect(mToggleMagnet, SIGNAL (toggled(bool)), this, SLOT (switchMagnetCallback(bool)));

    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Initialized connected callbacks of action buttons.");

    // Add visualization on map
    mUavMark = new UavMark(_mapPtr, "Uav_"+std::to_string(mUavId));
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Created mark on map.");

    // Start updating Gui
    mRunGui = true;
    mGuiThread = std::thread([&](){
        while(mRunGui){
            updateGui();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    });
}

//---------------------------------------------------------------------------------------------------------------------
UavInterface::~UavInterface() {
    mRunGui = false;
    if(mGuiThread.joinable()){
        mGuiThread.join();
    }
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::takeOffCallback(){
    mTakeOffButton->setEnabled(false);
    mLandButton->setEnabled(false);
    mTakeOffThread = new std::thread([this](){
        ros::NodeHandle nh;
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Sending take off service.");
        ros::ServiceClient client = nh.serviceClient<uav_state_machine::takeoff_service>("/mbzirc_"+std::to_string(mUavId)+"/uav_state_machine/takeoff");
        uav_state_machine::takeoff_service call;
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
        ros::ServiceClient client = nh.serviceClient<uav_state_machine::land_service>("/mbzirc_"+std::to_string(mUavId)+"/uav_state_machine/land");
        uav_state_machine::land_service call;
        auto res = client.call(call);
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Returned call with result: "+std::to_string(res));
        mLandButton->setEnabled(true);
        mTakeOffButton->setEnabled(true);
    });

}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::targetCallback(){
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<uav_state_machine::target_service>("/mbzirc_"+std::to_string(mUavId)+"/uav_state_machine/enabled");
    uav_state_machine::target_service call;
    call.request.enabled = mTargetEnable->isChecked();
    call.request.color = mColorSpin->value();
    //call.request.shape = mShapeSpin->value();
    call.request.global_position.x =  mHalPose.position[0];
    call.request.global_position.y =  mHalPose.position[1];
    call.request.global_position.z =  mHalPose.position[2];  // Fixed z? 0.0m?
    call.request.target_id = 1;

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
        ros::ServiceClient client = nh.serviceClient<uav_state_machine::magnetize_service>("/mbzirc_"+std::to_string(mUavId)+"/catching_device/magnetize");
        uav_state_machine::magnetize_service srv;
        srv.request.magnetize = _state;
        if (client.call(srv)) {
            mToggleMagnet->setEnabled(true);
        } else {
            LogManager::get()->status("UAV_"+std::to_string(mUavId), "Failed to switch magnet.");
        }
    });
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::addWpButtonCallback(){
    double x = mWpX->text().toFloat();
    double y = mWpY->text().toFloat();
    double z = mWpZ->text().toFloat();

    mWaypoints.push_back({x, y, z});
    std::stringstream ss;
    ss << "WP: [" << x << ", " << y << ", "<< z << "]";
    mWaypointListBox->addItem(ss.str().c_str());

}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::eraseWpButtonCallback() {
    int index = mWaypointListBox->currentIndex();
    mWaypointListBox->removeItem(index);
    mWaypoints.erase(mWaypoints.begin()+index);
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::sendWpButtonCallback(){
    mSendWpList->setEnabled(false);
    mAddWpButton->setEnabled(false);
    mEraseWpButton->setEnabled(false);
    mTakeOffThread = new std::thread([this](){
        ros::NodeHandle nh;
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Sending "+std::to_string(mWaypoints.size())+" wps.");
        ros::ServiceClient client = nh.serviceClient<uav_state_machine::waypoint_service>("/mbzirc_"+std::to_string(mUavId)+"/uav_state_machine/waypoint");
        uav_state_machine::waypoint_service req;
        req.request.action = mWpAction->value();
        for(auto wp: mWaypoints){
            geometry_msgs::Point p;
            p.x = wp[0];
            p.y = wp[1];
            p.z = wp[2];
            req.request.waypoint_track.push_back(p);
        }

        auto res = client.call(req);
        LogManager::get()->status("UAV_"+std::to_string(mUavId), "Returned call with result: "+std::to_string(res));
        mSendWpList->setEnabled(true);
        mAddWpButton->setEnabled(true);
        mEraseWpButton->setEnabled(true);
    });
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::altitudeCallback(const std_msgs::Float64ConstPtr &_msg) {
    mAltitude = _msg->data;
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::geodesicCallback(const sensor_msgs::NavSatFixConstPtr &_msg) {
    mLatitude = _msg->latitude;
    mLongitude = _msg->longitude;
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::halPoseCallback(const std_msgs::String::ConstPtr& _msg) {
    std::stringstream msg;
    msg << _msg->data;
    msg >> mHalPose;
    //std::cout << "mHalPose: " << mHalPose.position[0] << ", " << mHalPose.position[1] << ", " << mHalPose.position[2] << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::rcMagnetInterruptorCallback(const std_msgs::BoolConstPtr &_msg) {
    mInterruptorState = _msg->data;
}

//---------------------------------------------------------------------------------------------------------------------
void UavInterface::updateGui() {
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updating display altitude: " + std::to_string(mAltitude));
    mAltitudeBox->display(mAltitude);

    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updating display longitude: " + std::to_string(mLongitude));
    mLongitudeBox->display(mLongitude);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updating display latitude: " + std::to_string(mLatitude));
    mLatitudeBox->display(mLatitude);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updating position in map");
    //mUavMark->newPosition(mLongitude, mLatitude);
    LogManager::get()->status("UAV_"+std::to_string(mUavId), "Updated position in map");

    mInterruptorLed->setState(mInterruptorState);
}
