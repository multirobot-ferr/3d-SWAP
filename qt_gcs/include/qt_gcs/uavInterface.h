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

class UavInterface : public QGroupBox {
    Q_OBJECT

public:
    UavInterface(int _argc, char **_argv, int _index);
    ~UavInterface();

private slots:
    void takeOffCallback();
    void targetCallback();

    void altitudeCallback(const std_msgs::Float64ConstPtr &_msg);
    void geodesicCallback(const sensor_msgs::NavSatFixConstPtr &_msg);
private:
    int mUavId;
    QHBoxLayout *mMainLayoutUav;

    QVBoxLayout *mOdometryLayoutUav;
    QLCDNumber *mAltitudeBox;
    ros::Subscriber mAltitudeSubscriber;
    QLCDNumber *mLatitudeBox;
    QLCDNumber *mLongitudeBox;
    ros::Subscriber mGeodesicSubscriber;

    QVBoxLayout *mActionsLayoutUav;
    QHBoxLayout *mTakeOffLayout;
    QPushButton *mTakeOffButton;
    QDoubleSpinBox *mTakeOffAltitude;

    QHBoxLayout *mTargetLayout;
    QPushButton *mTargetButton;
    QRadioButton *mTargetEnable;
    QSpinBox *mColorSpin;
    QSpinBox *mShapeSpin;

};

#endif
