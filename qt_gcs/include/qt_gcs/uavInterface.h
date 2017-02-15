///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///
///
///
///
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _QTGCS_UAVINTERFACE_H_
#define _QTGCS_UAVINTERFACE_H_

#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QRadioButton>
#include <QLCDNumber>
#include <QComboBox>
#include <QIntValidator>
#include <QLineEdit>

#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/server.h>


#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>

#include <marble/MarbleWidget.h>
#include <thread>

#include <qt_gcs/UavMark.h>
#include <qt_gcs/LedIndicator.h>
#include <uav_state_machine/waypoint_service.h>

class UavInterface : public QGroupBox {
    Q_OBJECT

public:
    UavInterface(int _argc, char **_argv, int _index,  Marble::MarbleWidget* _mapPtr);
    ~UavInterface();

private slots:
    void takeOffCallback();
    void landCallback();
    void targetCallback();
    void centerCallback();
    void switchMagnetCallback(bool _state);
    void addWpButtonCallback();
    void eraseWpButtonCallback();
    void sendWpButtonCallback();

private:
    void altitudeCallback(const std_msgs::Float64ConstPtr &_msg);
    void geodesicCallback(const sensor_msgs::NavSatFixConstPtr &_msg);
    void rcMagnetInterruptorCallback(const std_msgs::BoolConstPtr &_msg);

    void updateGui();

private:
    int mUavId;
    QHBoxLayout *mMainLayoutUav;

    // Odometry items ------
    QVBoxLayout *mOdometryLayoutUav;
    QLCDNumber *mAltitudeBox;
    ros::Subscriber mAltitudeSubscriber;
    QLCDNumber *mLatitudeBox;
    QLCDNumber *mLongitudeBox;
    ros::Subscriber mGeodesicSubscriber;
    double mAltitude, mLongitude, mLatitude;

    // Action items --------
    QVBoxLayout *mActionsLayoutUav;
    // Take off
    QHBoxLayout *mTakeOffLayout;
    QPushButton *mTakeOffButton;
    QDoubleSpinBox *mTakeOffAltitude;
    std::thread *mTakeOffThread;

    // Land
    QHBoxLayout *mLandLayout;
    QPushButton *mLandButton;
    std::thread *mLandThread;

    // Target
    QHBoxLayout *mTargetLayout;
    QPushButton *mTargetButton;
    QRadioButton *mTargetEnable;
    QSpinBox *mColorSpin;
    QSpinBox *mShapeSpin;

    // Magnet
    QPushButton *mToggleMagnet;
    std::thread *mToggleMagnetThread;
    LedIndicator *mInterruptorLed;
    ros::Subscriber mMagnetSubscriber;
    bool mInterruptorState;

    // Waypoints
    QComboBox *mWaypointListBox;
    QPushButton *mEraseWpButton;
    QPushButton *mAddWpButton;
    QPushButton *mSendWpList;
    QSpinBox *mWpAction;
    QLineEdit *mWpX, *mWpY, *mWpZ;
    std::vector<std::array<double,3>> mWaypoints;
    QVBoxLayout mWaypointLayout;
    QHBoxLayout mWaypointListLayout, mWaypointEditLayout;

    // Center map on UAV
    QPushButton *mCenterTarget;

    // Uav map mark
    Marble::MarbleWidget *mMapPtr;
    UavMark *mUavMark;

    // GUI THREAD
    std::thread mGuiThread;
    bool mRunGui = false;

};

#endif
