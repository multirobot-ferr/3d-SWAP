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

#include <grvc_quadrotor_hal/types.h>
#include <grvc_quadrotor_hal/server.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/RCOut.h>

#include <marble/MarbleWidget.h>
#include <thread>

#include <qt_gcs/UavMark.h>
#include <qt_gcs/LedIndicator.h>

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

    void altitudeCallback(const std_msgs::Float64ConstPtr &_msg);
    void geodesicCallback(const sensor_msgs::NavSatFixConstPtr &_msg);
    void magnetInterruptorCallback(const mavros_msgs::RCOutConstPtr &_msg);
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
    LedIndicator *mMagnetLed;
    ros::Subscriber mMagnetSubscriber;

    // Center map on UAV
    QPushButton *mCenterTarget;

    // Uav map mark
    Marble::MarbleWidget *mMapPtr;
    UavMark *mUavMark;

};

#endif
